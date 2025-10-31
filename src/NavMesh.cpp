#include "NavMesh.h"

NavMesh::NavMesh() {
	// Constructor implementation
}

NavMesh::~NavMesh()
{
	_cleanup();
}

std::vector<irr::core::vector3df> NavMesh::getPath(irr::core::vector3df start, irr::core::vector3df end)
{
    return _returnPath(start, end);
}

void NavMesh::renderDebugPath(std::vector<irr::core::vector3df> path, video::IVideoDriver* driver)
{
    for (int i = 0; i < path.size(); i++) {
        if ((i + 1) == (int)path.size()) {
            break;
        }
        video::SMaterial m;
        m.Lighting = false;
        driver->setMaterial(m);
        driver->setTransform(video::ETS_WORLD, core::matrix4());
        core::vector3df n1 = core::vector3df(path[i].X, path[i].Y, path[i].Z);
        core::vector3df n2 = core::vector3df(path[i + 1].X, path[i + 1].Y, path[i + 1].Z);
        driver->draw3DLine(n1, n2, video::SColor(255, 0, 0, 255));
    };
}

void NavMesh::_cleanup()
{
	if (_verts) { delete[] _verts; _verts = 0; }
	if (_tris) { delete[] _tris; _tris = 0; }

	delete[] _triareas;
	
	_triareas = 0;
	rcFreeHeightField(_solid);
	_solid = 0;
	rcFreeCompactHeightfield(_chf);
	_chf = 0;
	rcFreeContourSet(_cset);
	_cset = 0;
	rcFreePolyMesh(_pmesh);
	_pmesh = 0;
	rcFreePolyMeshDetail(_dmesh);
	_dmesh = 0;
}

bool NavMesh::_handleBuild(irr::scene::IMeshSceneNode* node)
{
    _cleanup();

    std::vector<float> vs;
    std::vector<int> ts;
    int nverts;
    int ntris;

    if (!this->_getMeshBufferData(node, vs, nverts, ts, ntris))
    {
        printf("getMeshBufferData() failed!\n");
        return false;
    }

    // populate verts and tris:
    _verts = new float[nverts];
    for (int n = 0; n < nverts; ++n)
    {
        _verts[n] = vs[n];
    }

    _tris = new int[ntris * 3];
    for (int n = 0; n < ntris; ++n)
    {
        _tris[n * 3] = ts[n * 3];
        _tris[n * 3 + 1] = ts[n * 3 + 1];
        _tris[n * 3 + 2] = ts[n * 3 + 2];
    }
    printf("vs.size()=%i,nverts=%i\n", vs.size(), nverts);
    printf("ts.size()=%i,ntris=%i\n", ts.size(), ntris);
    float bmin[3];
    float bmax[3];

    rcCalcBounds(_verts, nverts / 3, bmin, bmax);

    //
    // Step 1. Initialize build config.
    //

    // Init build configuration from GUI
    memset(&_cfg, 0, sizeof(_cfg));
    _cfg.cs = _cellSize;
    _cfg.ch = _cellHeight;
    _cfg.walkableSlopeAngle = _agentMaxSlope;
    _cfg.walkableHeight = (int)ceilf(_agentHeight / _cfg.ch);
    _cfg.walkableClimb = (int)floorf(_agentMaxClimb / _cfg.ch);
    _cfg.walkableRadius = (int)ceilf(_agentRadius / _cfg.cs);
    _cfg.maxEdgeLen = (int)(_edgeMaxLen / _cellSize);
    _cfg.maxSimplificationError = _edgeMaxError;
    _cfg.minRegionArea = (int)rcSqr(_regionMinSize);      // Note: area = size*size
    _cfg.mergeRegionArea = (int)rcSqr(_regionMergeSize);  // Note: area = size*size
    _cfg.maxVertsPerPoly = (int)_vertsPerPoly;
    _cfg.detailSampleDist = _detailSampleDist < 0.9f ? 0 : _cellSize * _detailSampleDist;
    _cfg.detailSampleMaxError = _cellHeight * _detailSampleMaxError;

    // Set the area where the navigation will be build.
    // Here the bounds of the input mesh are used, but the
    // area could be specified by an user defined box, etc.
    rcVcopy(_cfg.bmin, bmin);
    rcVcopy(_cfg.bmax, bmax);
    rcCalcGridSize(_cfg.bmin, _cfg.bmax, _cfg.cs, &_cfg.width, &_cfg.height);

    // Reset build times gathering.
    _ctx->resetTimers();

    // Start the build process.
    _ctx->startTimer(RC_TIMER_TOTAL);

    _ctx->log(RC_LOG_PROGRESS, "Building navigation:");
    _ctx->log(RC_LOG_PROGRESS, " - %d x %d cells", _cfg.width, _cfg.height);
    _ctx->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", nverts / 1000.0f, ntris / 1000.0f);

    //
    // Step 2. Rasterize input polygon soup.
    //

    // Allocate voxel heightfield where we rasterize our input data to.
    _solid = rcAllocHeightfield();
    if (!_solid)
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
        return false;
    }
    if (!rcCreateHeightfield(_ctx, *_solid, _cfg.width, _cfg.height, _cfg.bmin, _cfg.bmax, _cfg.cs, _cfg.ch))
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
        return false;
    }

    // Allocate array that can hold triangle area types.
    // If you have multiple meshes you need to process, allocate
    // and array which can hold the max number of triangles you need to process.
    _triareas = new unsigned char[ntris];
    if (!_triareas)
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory '_triareas' (%d).", ntris);
        return false;
    }

    // Find triangles which are walkable based on their slope and rasterize them.
    // If your input data is multiple meshes, you can transform them here, calculate
    // the are type for each of the meshes and rasterize them.
    memset(_triareas, 0, ntris * sizeof(unsigned char));
    rcMarkWalkableTriangles(_ctx, _cfg.walkableSlopeAngle, _verts, nverts, _tris, ntris, _triareas);
    rcRasterizeTriangles(_ctx, _verts, nverts, _tris, _triareas, ntris, *_solid, _cfg.walkableClimb);

    this->_showHeightFieldInfo(*_solid); //<-----------------------------------------------------------------------------------------

    if (!_keepInterResults)
    {
        delete[] _triareas;
        _triareas = 0;
    }

    //
    // Step 3. Filter walkables surfaces.
    //

    // Once all geoemtry is rasterized, we do initial pass of filtering to
    // remove unwanted overhangs caused by the conservative rasterization
    // as well as filter spans where the character cannot possibly stand.
    rcFilterLowHangingWalkableObstacles(_ctx, _cfg.walkableClimb, *_solid);
    rcFilterLedgeSpans(_ctx, _cfg.walkableHeight, _cfg.walkableClimb, *_solid);
    rcFilterWalkableLowHeightSpans(_ctx, _cfg.walkableHeight, *_solid);


    //
    // Step 4. Partition walkable surface to simple regions.
    //

    // Compact the heightfield so that it is faster to handle from now on.
    // This will result more cache coherent data as well as the neighbours
    // between walkable cells will be calculated.
    _chf = rcAllocCompactHeightfield();
    if (!_chf)
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
        return false;
    }
    if (!rcBuildCompactHeightfield(_ctx, _cfg.walkableHeight, _cfg.walkableClimb, *_solid, *_chf))
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
        return false;
    }

    if (!_keepInterResults)
    {
        rcFreeHeightField(_solid);
        _solid = 0;
    }

    // Erode the walkable area by agent radius.
    if (!rcErodeWalkableArea(_ctx, _cfg.walkableRadius, *_chf))
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
        return false;
    }

    // (Optional) Mark areas.
    //const ConvexVolume* vols = m_geom->getConvexVolumes();
    //for (int i  = 0; i < m_geom->getConvexVolumeCount(); ++i)
    //rcMarkConvexPolyArea(_ctx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *_chf);

    if (_monotonePartitioning)
    {
        // Partition the walkable surface into simple regions without holes.
        // Monotone partitioning does not need distancefield.
        if (!rcBuildRegionsMonotone(_ctx, *_chf, 0, _cfg.minRegionArea, _cfg.mergeRegionArea))
        {
            _ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build regions.");
            return false;
        }
    }
    else
    {
        // Prepare for region partitioning, by calculating distance field along the walkable surface.
        if (!rcBuildDistanceField(_ctx, *_chf))
        {
            _ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
            return false;
        }

        // Partition the walkable surface into simple regions without holes.
        if (!rcBuildRegions(_ctx, *_chf, 0, _cfg.minRegionArea, _cfg.mergeRegionArea))
        {
            _ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build regions.");
            return false;
        }
    }

    //
    // Step 5. Trace and simplify region contours.
    //

    // Create contours.
    _cset = rcAllocContourSet();
    if (!_cset)
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
        return false;
    }
    if (!rcBuildContours(_ctx, *_chf, _cfg.maxSimplificationError, _cfg.maxEdgeLen, *_cset))
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
        return false;
    }
    printf("_cset->nconts=%i\n", _cset->nconts);
    //
    // Step 6. Build polygons mesh from contours.
    //

    // Build polygon navmesh from the contours.
    _pmesh = rcAllocPolyMesh();
    if (!_pmesh)
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
        return false;
    }
    if (!rcBuildPolyMesh(_ctx, *_cset, _cfg.maxVertsPerPoly, *_pmesh))
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
        return false;
    }

    //
    // Step 7. Create detail mesh which allows to access approximate height on each polygon.
    //

    _dmesh = rcAllocPolyMeshDetail();
    if (!_dmesh)
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
        return false;
    }

    if (!rcBuildPolyMeshDetail(_ctx, *_pmesh, *_chf, _cfg.detailSampleDist, _cfg.detailSampleMaxError, *_dmesh))
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
        return false;
    }

    if (!_keepInterResults)
    {
        rcFreeCompactHeightfield(_chf);
        _chf = 0;
        rcFreeContourSet(_cset);
        _cset = 0;
    }


    // At this point the navigation mesh data is ready, you can access it from _pmesh.
    // See duDebugDrawPolyMesh or dtCreateNavMeshData as examples how to access the data.
    //
    // (Optional) Step 8. Create Detour data from Recast poly mesh.
    //
    // ...

    //
    // (Optional) Step 8. Create Detour data from Recast poly mesh.
    //

    // The GUI may allow more max points per polygon than Detour can handle.
    // Only build the detour navmesh if we do not exceed the limit.
    if (_cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
    {
        unsigned char* navData = 0;
        int navDataSize = 0;

        // Update poly flags from areas.
        for (int i = 0; i < _pmesh->npolys; ++i)
        {
            if (_pmesh->areas[i] == RC_WALKABLE_AREA)
                _pmesh->areas[i] = SAMPLE_POLYAREA_GROUND;

            if (_pmesh->areas[i] == SAMPLE_POLYAREA_GROUND ||
                _pmesh->areas[i] == SAMPLE_POLYAREA_GRASS ||
                _pmesh->areas[i] == SAMPLE_POLYAREA_ROAD)
            {
                _pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
            }
            else if (_pmesh->areas[i] == SAMPLE_POLYAREA_WATER)
            {
                _pmesh->flags[i] = SAMPLE_POLYFLAGS_SWIM;
            }
            else if (_pmesh->areas[i] == SAMPLE_POLYAREA_DOOR)
            {
                _pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
            }
        }

        //  InputGeom* geom = new InputGeom;


            //m_geom->raycastMesh()
        dtNavMeshCreateParams params;
        memset(&params, 0, sizeof(params));
        params.verts = _pmesh->verts;
        params.vertCount = _pmesh->nverts;
        params.polys = _pmesh->polys;
        params.polyAreas = _pmesh->areas;
        params.polyFlags = _pmesh->flags;
        params.polyCount = _pmesh->npolys;
        params.nvp = _pmesh->nvp;
        params.detailMeshes = _dmesh->meshes;
        params.detailVerts = _dmesh->verts;
        params.detailVertsCount = _dmesh->nverts;
        params.detailTris = _dmesh->tris;
        params.detailTriCount = _dmesh->ntris;

        params.offMeshConAreas = 0;
        params.offMeshConCount = 0;
        params.offMeshConDir = 0;
        params.offMeshConFlags = 0;
        params.offMeshConRad = 0;
        params.offMeshConUserID = 0;
        params.offMeshConVerts = 0;

        params.walkableHeight = _agentHeight;
        params.walkableRadius = _agentRadius;
        params.walkableClimb = _agentMaxClimb;
        rcVcopy(params.bmin, _pmesh->bmin);
        rcVcopy(params.bmax, _pmesh->bmax);
        params.cs = _cfg.cs;
        params.ch = _cfg.ch;
        params.buildBvTree = true;

        if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
        {
            _ctx->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
            return false;
        }

        _navMesh = dtAllocNavMesh();
        if (!_navMesh)
        {
            dtFree(navData);
            _ctx->log(RC_LOG_ERROR, "Could not create Detour navmesh");
            return false;
        }


        dtStatus status;

        status = _navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
        if (dtStatusFailed(status))
        {
            dtFree(navData);
            _ctx->log(RC_LOG_ERROR, "Could not init Detour navmesh");
            return false;
        }

        _navQuery = dtAllocNavMeshQuery();
        status = _navQuery->init(_navMesh, 2048);
        if (dtStatusFailed(status))
        {
            _ctx->log(RC_LOG_ERROR, "Could not init Detour navmesh query");
            return false;
        }

    }


    _ctx->stopTimer(RC_TIMER_TOTAL);

    // Show performance stats.
    _ctx->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", _pmesh->nverts, _pmesh->npolys);

    _totalBuildTimeMs = _ctx->getAccumulatedTime(RC_TIMER_TOTAL) / 1000.0f;

    // irr part:
    printf("RC_LOG_PROGRESS >> Polymesh: %d vertices  %d polygons\n", _pmesh->nverts, _pmesh->npolys);
    return true;
}

void NavMesh::_resetCommonSettings()
{
    _cellSize = 0.2f;
    _cellHeight = 0.2f;
    _agentHeight = 2.0f;
    _agentRadius = 0.2f;
    _agentMaxClimb = 0.9f;
    _agentMaxSlope = 45.0f;
    _regionMinSize = 8;
    _regionMergeSize = 20;
    _monotonePartitioning = false;
    _edgeMaxLen = 12.0f;
    _edgeMaxError = 1.3f;
    _vertsPerPoly = 6.0f;
    _detailSampleDist = 6.0f;
    _detailSampleMaxError = 1.0f;
}

bool NavMesh::_getMeshBufferData(irr::scene::IMeshSceneNode* node, std::vector<float>& verts, int& nverts, std::vector<int>& tris, int& ntris)
{
    if (!node)
    {
        printf("_getMeshBufferData: node is null\n");
        return false;
    }
    scene::IMesh* mesh = node->getMesh();
    if (!mesh)
    {
        printf("_getMeshBufferData: mesh is null\n");
        return false;
    }
    // Get the node's world transformation matrix
    core::matrix4 transform = node->getAbsoluteTransformation();
    nverts = 0;
    ntris = 0;
    int vertexOffset = 0; // Offset for indices as we add vertices from multiple buffers
    // Iterate over ALL mesh buffers
    for (u32 i = 0; i < mesh->getMeshBufferCount(); ++i)
    {
        scene::IMeshBuffer* buffer = mesh->getMeshBuffer(i);
        int currentVertCount = buffer->getVertexCount();
        int currentTriCount = buffer->getIndexCount() / 3;
        if ((currentVertCount <= 0) || (currentTriCount <= 0))
        {
            continue; // Skip empty buffers
        }
        // Add vertices
        for (int j = 0; j < currentVertCount; ++j)
        {
            core::vector3df pos = buffer->getPosition(j);
            // Apply world transformation to the vertex
            transform.transformVect(pos);
            verts.push_back(pos.X);
            verts.push_back(pos.Y);
            verts.push_back(pos.Z);
        }
        // Add triangles (indices)
        u16* indices = buffer->getIndices();
        for (int j = 0; j < currentTriCount * 3; ++j)
        {
            // Add the vertexOffset so indices point to the correct vertices
            // in our combined vertex list
            tris.push_back(indices[j] + vertexOffset);
        }
        nverts += currentVertCount;
        ntris += currentTriCount;
        vertexOffset += currentVertCount; // Update the offset for the next buffer
    }
    if ((nverts <= 0) || (ntris <= 0))
    {
        printf("_getMeshBufferData: No vertices or triangles found in any buffer.\n");
        return false;
    }
    // Update nverts to be the float count (X, Y, Z), as expected by Recast
    nverts *= 3;
    printf("_getMeshBufferData: Total nverts (components)=%i\n", nverts);
    printf("_getMeshBufferData: Total ntris=%i\n", ntris);
    return true;
}

bool NavMesh::_setupIrrSMeshFromRecastDetailMesh(irr::scene::SMesh* smesh)
{
    rcPolyMeshDetail* dmesh = _dmesh;
    if ((!smesh) || (!dmesh))
        return false;
    scene::SMeshBuffer* buffer = new scene::SMeshBuffer();
    std::vector<float> vertsOut;
    int nvertsOut = 0;
    std::vector<int> trisOut;
    int ntrisOut = 0;
    bool ok = _getMeshDataFromPolyMeshDetail
    (
        dmesh,
        vertsOut, nvertsOut,
        trisOut, ntrisOut
    );
    if (!ok)
    {
        printf("_setupIrrSMeshFromRecastDetailMesh(): _getMeshDataFromPolyMeshDetail() failed!\n");
        buffer->drop();
        return false;
    }
    printf("_setupIrrSMeshFromRecastDetailMesh(): vertsOut.size()=%i\n", vertsOut.size());
    ok = _setMeshBufferData
    (
        *buffer, vertsOut, nvertsOut, trisOut, ntrisOut
    );
    if (!ok)
    {
        printf("_setupIrrSMeshFromRecastDetailMesh(): _setMeshBufferData() failed!\n");
        buffer->drop();
        return false;
    }
    // this is important:
    // if not done, scene may disappear sometimes!
    // recalculate bounding box
    for (u32 n = 0; n < buffer->getVertexCount(); ++n)
    {
        buffer->BoundingBox.addInternalPoint(buffer->Vertices[n].Pos);
    }
    smesh->addMeshBuffer(buffer);
    smesh->setHardwareMappingHint(scene::EHM_STATIC, scene::EBT_VERTEX_AND_INDEX);
    smesh->recalculateBoundingBox();
    buffer->drop();
    return true;
}

void NavMesh::_showHeightFieldInfo(const rcHeightfield& hf)
{
    int w = hf.width;
    int h = hf.height;
    const float* bmin = hf.bmin;
    const float* bmax = hf.bmax;
    float cs = hf.cs;
    float ch = hf.ch;

    printf("rcHeightfield hf: w=%i,h=%i,bmin=(%f,%f,%f),bmax=(%f,%f,%f),cs=%f,ch=%f\n", w, h, bmin[0], bmin[1], bmin[2], bmax[0], bmax[1], bmax[2], cs, ch);
}

bool NavMesh::_getMeshDataFromPolyMeshDetail(rcPolyMeshDetail* dmesh, std::vector<float>& vertsOut, int& nvertsOut, std::vector<int>& trisOut, int& ntrisOut)
{
    printf("getMeshDataFromPolyMeshDetail(): dmesh->nmeshes=%i\n", dmesh->nmeshes);
    if (dmesh->nmeshes == 0)
    {
        printf("getMeshDataFromPolyMeshDetail(): dmesh->nmeshes == 0\n");
        return false;
    }

    nvertsOut = dmesh->nverts * 3;
    printf("getMeshDataFromPolyMeshDetail: nvertsOut=%i\n", nvertsOut);

    ntrisOut = dmesh->ntris;
    printf("getMeshDataFromPolyMeshDetail: ntrisOut=%i\n", ntrisOut);

    const float* verts = dmesh->verts;

    for (int n = 0; n < nvertsOut; ++n)
    {
        vertsOut.push_back(verts[n]);
    }


    //FIXME:
    int tri_offset = 0;
    int old_nverts = 0;
    for (int p = 0; p < dmesh->nmeshes; ++p)
    {
        unsigned int* m = &(dmesh->meshes[p * 4]);
        const unsigned short bverts = m[0];
        const unsigned short nverts = m[1];
        const unsigned short btris = m[2];
        const unsigned short ntris = m[3];
        const unsigned char* tris = &(dmesh->tris[btris * 4]);

        tri_offset += old_nverts;

        for (int n = 0; n < ntris; ++n)
        {
            for (int k = 0; k < 3; ++k)
            {

                int tri = tris[n * 4 + k] + tri_offset;
                trisOut.push_back(tri);
            }
        }
        old_nverts = nverts;
    }

    return true;

}

bool NavMesh::_setMeshBufferData(irr::scene::SMeshBuffer& buffer, const std::vector<float>& verts, int& nverts, const std::vector<int>& tris, int& ntris)
{
    //TODO:
    // err handling:
    if (verts.empty() || tris.empty())
    {
        return false;
    }

    buffer.Vertices.set_used(nverts / 3);
    buffer.Indices.set_used(ntris * 3); // ntris triangles!

    for (int i = 0; i < nverts; i += 3)
    {
        float x = verts[i];
        float y = verts[i + 1];
        float z = verts[i + 2];

        buffer.Vertices[i / 3] = video::S3DVertex
        (
            x, y, z,
            0.0f, 1.0f, 0.0f,
            0x8000FF00,
            0.0f, 0.0f
        );
    }


    for (int i = 0; i < ntris * 3; ++i)
    {
        int index = tris[i];
        buffer.Indices[i] = index;
    }


    buffer.getMaterial().Lighting = false;
    buffer.getMaterial().BackfaceCulling = false;
    buffer.getMaterial().Wireframe = true;
    buffer.getMaterial().Thickness = 2.0f;
    buffer.getMaterial().MaterialType = video::EMT_TRANSPARENT_ALPHA_CHANNEL;
    buffer.recalculateBoundingBox();

    return true;
}

std::vector<irr::core::vector3df> NavMesh::_returnPath(irr::core::vector3df pStart, irr::core::vector3df pEnd)
{
    std::vector<irr::core::vector3df> lstPoints;
    if (_navQuery)
    {
        if (_navMesh == 0)
        {
            return lstPoints;
        }
        dtQueryFilter filter;
        dtPolyRef startRef;
        dtPolyRef endRef;
        const int MAX_POLYS = 256;
        dtPolyRef polys[MAX_POLYS];
        dtPolyRef returnedPath[MAX_POLYS];
        float straightPath[MAX_POLYS * 3];
        int numStraightPaths;
        float spos[3] = { pStart.X, pStart.Y, pStart.Z };
        float epos[3] = { pEnd.X, pEnd.Y, pEnd.Z };
        float polyPickExt[3];
        polyPickExt[0] = 2;
        polyPickExt[1] = 4;
        polyPickExt[2] = 2;
        _navQuery->findNearestPoly(spos, polyPickExt, &filter, &startRef, 0);
        if (startRef == 0)
        {
            return lstPoints;
        }
        _navQuery->findNearestPoly(epos, polyPickExt, &filter, &endRef, 0);
        if (endRef == 0)
        {
            return lstPoints;
        }
        dtStatus findStatus = DT_FAILURE;
        int pathCount;
        findStatus = _navQuery->findPath(startRef, endRef, spos, epos, &filter, returnedPath, &pathCount, MAX_POLYS);
        if (pathCount > 0)
        {
            findStatus = _navQuery->findStraightPath(spos, epos, returnedPath,
                pathCount, straightPath, 0, 0, &numStraightPaths, MAX_POLYS);
            for (int i = 0; i < numStraightPaths; ++i)
            {
                vector3df cpos(straightPath[i * 3], straightPath[i * 3 + 1] + 0.25,
                    straightPath[i * 3 + 2]);
                lstPoints.push_back(cpos);
            }
        }
    }
    return lstPoints;
}
