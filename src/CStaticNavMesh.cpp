#include "IrrRecastDetour/CStaticNavMesh.h"
#include <irrlicht.h> // For Irrlicht types

// Use explicit namespaces
using irr::core::vector3df;
using irr::core::matrix4;
using irr::scene::ISceneNode;
using irr::scene::IMeshSceneNode;

CStaticNavMesh::CStaticNavMesh(irr::scene::ISceneNode* parent, irr::scene::ISceneManager* mgr, irr::s32 id)
    : INavMesh(parent, mgr, id), // Call the base constructor
    _ctx(new rcContext()),
    _totalBuildTimeMs(0.0f)
{
    // Constructor logic specific to StaticNavMesh (if any) goes here.
    // _ctx is initialized in the member initializer list above.
}

CStaticNavMesh::~CStaticNavMesh()
{
    // All smart pointers handle their own cleanup.
}

bool CStaticNavMesh::build(IMeshSceneNode* levelNode, const NavMeshParams& params)
{
    if (!levelNode)
    {
        printf("ERROR: StaticNavMesh::build: levelNode is null.\n");
        return false;
    }

    _params = params;
    _totalBuildTimeMs = 0.0f;

    // Store defaults for the base class to use when creating agents
    _defaultAgentRadius = _params.AgentRadius;
    _defaultAgentHeight = _params.AgentHeight;

    // Clear all previous build data
    // The .reset() calls will trigger the custom deleters on any existing data.
    _solid.reset();
    _chf.reset();
    _cset.reset();
    _pmesh.reset();
    _dmesh.reset();

    // Clear Detour objects (in base class)
    _navMesh.reset();
    _navQuery.reset();
    _crowd.reset();


    // Remove old debug mesh if it exists
    if (_naviDebugData)
    {
        _naviDebugData->remove();
        _naviDebugData = nullptr;
    }

    //
    // Step 1. Extract Irrlicht geometry
    //
    _verts.clear();
    _tris.clear();
    if (!this->_getMeshBufferData(levelNode, _verts, _tris))
    {
        printf("ERROR: StaticNavMesh::build: _getMeshBufferData() failed.\n");
        return false;
    }

    int nverts = _verts.size() / 3;
    int ntris = _tris.size() / 3;

    if (nverts == 0 || ntris == 0)
    {
        printf("ERROR: StaticNavMesh::build: No geometry found in levelNode.\n");
        return false;
    }

    float bmin[3], bmax[3];
    rcCalcBounds(_verts.data(), nverts, bmin, bmax);

    // Set the ISceneNode's bounding box (from base class)
    _box.reset(irr::core::vector3df(bmin[0], bmin[1], bmin[2]));
    _box.addInternalPoint(irr::core::vector3df(bmax[0], bmax[1], bmax[2]));


    //
    // Step 2. Initialize build config.
    //
    memset(&_cfg, 0, sizeof(_cfg));
    _cfg.cs = _params.CellSize;
    _cfg.ch = _params.CellHeight;
    _cfg.walkableSlopeAngle = _params.AgentMaxSlope;
    _cfg.walkableHeight = (int)ceilf(_params.AgentHeight / _cfg.ch);
    _cfg.walkableClimb = (int)floorf(_params.AgentMaxClimb / _cfg.ch);
    _cfg.walkableRadius = (int)ceilf(_params.AgentRadius / _cfg.cs);
    _cfg.maxEdgeLen = (int)(_params.EdgeMaxLen / _cfg.cs);
    _cfg.maxSimplificationError = _params.EdgeMaxError;
    _cfg.minRegionArea = (int)rcSqr(_params.RegionMinSize);
    _cfg.mergeRegionArea = (int)rcSqr(_params.RegionMergeSize);
    _cfg.maxVertsPerPoly = (int)_params.VertsPerPoly;
    _cfg.detailSampleDist = _params.DetailSampleDist < 0.9f ? 0 : _cfg.cs * _params.DetailSampleDist;
    _cfg.detailSampleMaxError = _cfg.ch * _params.DetailSampleMaxError;

    rcVcopy(_cfg.bmin, bmin);
    rcVcopy(_cfg.bmax, bmax);
    rcCalcGridSize(_cfg.bmin, _cfg.bmax, _cfg.cs, &_cfg.width, &_cfg.height);

    _ctx->resetTimers();
    _ctx->startTimer(RC_TIMER_TOTAL);

    _ctx->log(RC_LOG_PROGRESS, "Building navigation:");
    _ctx->log(RC_LOG_PROGRESS, " - %d x %d cells", _cfg.width, _cfg.height);
    _ctx->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", nverts / 1000.0f, ntris / 1000.0f);

    //
    // Step 3. Rasterize input polygon soup.
    //
    _solid.reset(rcAllocHeightfield());
    if (!_solid)
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
        return false;
    }
    if (!rcCreateHeightfield(_ctx.get(), *_solid, _cfg.width, _cfg.height, _cfg.bmin, _cfg.bmax, _cfg.cs, _cfg.ch))
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
        return false;
    }

    _triareas.resize(ntris);

    memset(_triareas.data(), 0, ntris * sizeof(unsigned char));
    rcMarkWalkableTriangles(_ctx.get(), _cfg.walkableSlopeAngle, _verts.data(), nverts, _tris.data(), ntris, _triareas.data());
    rcRasterizeTriangles(_ctx.get(), _verts.data(), nverts, _tris.data(), _triareas.data(), ntris, *_solid, _cfg.walkableClimb);

    this->_showHeightFieldInfo(*_solid);

    if (!_params.KeepInterResults)
    {
        _triareas.clear();
    }

    //
    // Step 4. Filter walkables surfaces.
    //
    rcFilterLowHangingWalkableObstacles(_ctx.get(), _cfg.walkableClimb, *_solid);
    rcFilterLedgeSpans(_ctx.get(), _cfg.walkableHeight, _cfg.walkableClimb, *_solid);
    rcFilterWalkableLowHeightSpans(_ctx.get(), _cfg.walkableHeight, *_solid);

    //
    // Step 5. Partition walkable surface to simple regions.
    //
    _chf.reset(rcAllocCompactHeightfield());
    if (!_chf)
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
        return false;
    }
    if (!rcBuildCompactHeightfield(_ctx.get(), _cfg.walkableHeight, _cfg.walkableClimb, *_solid, *_chf))
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
        return false;
    }

    if (!_params.KeepInterResults)
    {
        _solid.reset(); // Frees the heightfield
    }

    if (!rcErodeWalkableArea(_ctx.get(), _cfg.walkableRadius, *_chf))
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
        return false;
    }

    if (_params.MonotonePartitioning)
    {
        if (!rcBuildRegionsMonotone(_ctx.get(), *_chf, 0, _cfg.minRegionArea, _cfg.mergeRegionArea))
        {
            _ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build regions.");
            return false;
        }
    }
    else
    {
        if (!rcBuildDistanceField(_ctx.get(), *_chf))
        {
            _ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
            return false;
        }
        if (!rcBuildRegions(_ctx.get(), *_chf, 0, _cfg.minRegionArea, _cfg.mergeRegionArea))
        {
            _ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build regions.");
            return false;
        }
    }

    //
    // Step 6. Trace and simplify region contours.
    //
    _cset.reset(rcAllocContourSet());
    if (!_cset)
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
        return false;
    }
    if (!rcBuildContours(_ctx.get(), *_chf, _cfg.maxSimplificationError, _cfg.maxEdgeLen, *_cset))
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
        return false;
    }
    printf("_cset->nconts=%i\n", _cset->nconts);

    //
    // Step 7. Build polygons mesh from contours.
    //
    _pmesh.reset(rcAllocPolyMesh());
    if (!_pmesh)
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
        return false;
    }
    if (!rcBuildPolyMesh(_ctx.get(), *_cset, _cfg.maxVertsPerPoly, *_pmesh))
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
        return false;
    }

    //
    // Step 8. Create detail mesh.
    //
    _dmesh.reset(rcAllocPolyMeshDetail());
    if (!_dmesh)
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
        return false;
    }
    if (!rcBuildPolyMeshDetail(_ctx.get(), *_pmesh, *_chf, _cfg.detailSampleDist, _cfg.detailSampleMaxError, *_dmesh))
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
        return false;
    }

    if (!_params.KeepInterResults)
    {
        _chf.reset();
        _cset.reset();
    }

    //
    // Step 9. Create Detour data from Recast poly mesh.
    //
    if (_cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
    {
        unsigned char* navData = nullptr;
        int navDataSize = 0;

        // Set polygon flags
        for (int i = 0; i < _pmesh->npolys; ++i)
        {
            if (_pmesh->areas[i] == RC_WALKABLE_AREA)
                _pmesh->areas[i] = (unsigned char)PolyAreas::GROUND;

            if (_pmesh->areas[i] == (unsigned char)PolyAreas::GROUND ||
                _pmesh->areas[i] == (unsigned char)PolyAreas::GRASS ||
                _pmesh->areas[i] == (unsigned char)PolyAreas::ROAD)
            {
                _pmesh->flags[i] = (unsigned short)PolyFlags::WALK;
            }
            else if (_pmesh->areas[i] == (unsigned char)PolyAreas::WATER)
            {
                _pmesh->flags[i] = (unsigned short)PolyFlags::SWIM;
            }
            else if (_pmesh->areas[i] == (unsigned char)PolyAreas::DOOR)
            {
                _pmesh->flags[i] = (unsigned short)PolyFlags::WALK | (unsigned short)PolyFlags::DOOR;
            }
        }

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

        params.walkableHeight = _params.AgentHeight;
        params.walkableRadius = _params.AgentRadius;
        params.walkableClimb = _params.AgentMaxClimb;
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

        // --- Initialize the BASE CLASS's _navMesh ---
        _navMesh.reset(dtAllocNavMesh());
        if (!_navMesh)
        {
            dtFree(navData);
            _ctx->log(RC_LOG_ERROR, "Could not create Detour navmesh");
            return false;
        }

        dtStatus status = _navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
        if (dtStatusFailed(status))
        {
            dtFree(navData);
            _ctx->log(RC_LOG_ERROR, "Could not init Detour navmesh");
            return false;
        }

        // --- Initialize the BASE CLASS's _navQuery ---
        _navQuery.reset(dtAllocNavMeshQuery());
        status = _navQuery->init(_navMesh.get(), 2048);
        if (dtStatusFailed(status))
        {
            _ctx->log(RC_LOG_ERROR, "Could not init Detour navmesh query");
            return false;
        }

        // --- Initialize the BASE CLASS's _crowd ---
        _crowd.reset(dtAllocCrowd());
        if (!_crowd)
        {
            _ctx->log(RC_LOG_ERROR, "Could not alloc crowd");
            return false;
        }

        if (!_crowd->init(MAX_AGENTS, _params.AgentRadius, _navMesh.get()))
        {
            _ctx->log(RC_LOG_ERROR, "Could not init crowd");
            return false;
        }
    }

    _ctx->stopTimer(RC_TIMER_TOTAL);
    _totalBuildTimeMs = _ctx->getAccumulatedTime(RC_TIMER_TOTAL) / 1000.0f;
    _ctx->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", _pmesh->nverts, _pmesh->npolys);

    return true;
}

ISceneNode* CStaticNavMesh::renderNavMesh()
{
    if (!SceneManager)
    {
        printf("ERROR: StaticNavMesh::renderNavMesh: SceneManager is null.\n");
        return nullptr;
    }
    if (!_dmesh)
    {
        printf("WARNING: StaticNavMesh::renderNavMesh: No detail mesh data to build from.\n");
        return nullptr;
    }

    // Remove old debug mesh if it exists
    if (_naviDebugData)
    {
        _naviDebugData->remove();
        _naviDebugData = nullptr;
    }

    irr::scene::SMesh* smesh = new irr::scene::SMesh();
    if (!_setupIrrSMeshFromRecastDetailMesh(smesh))
    {
        printf("ERROR: StaticNavMesh::renderNavMesh: _setupIrrSMeshFromRecastDetailMesh failed.\n");
        smesh->drop();
        return nullptr;
    }

    // Add the debug node as a child of this node
    _naviDebugData = SceneManager->addMeshSceneNode(smesh, this);

    if (_naviDebugData)
    {
        _naviDebugData->setName("NavMeshDebugNode");
        _naviDebugData->setMaterialFlag(irr::video::EMF_LIGHTING, false);
        _naviDebugData->setMaterialFlag(irr::video::EMF_WIREFRAME, true);
        _naviDebugData->setPosition(irr::core::vector3df(0, 0, 0));

        printf("Debug mesh created with %d vertices\n", smesh->getMeshBuffer(0)->getVertexCount());
    }

    this->setVisible(true);

    smesh->drop();
    return _naviDebugData;
}


// --- Private Helper Functions ---

bool CStaticNavMesh::_getMeshBufferData(
    IMeshSceneNode* node,
    std::vector<float>& verts,
    std::vector<int>& tris)
{
    if (!node) return false;
    irr::scene::IMesh* mesh = node->getMesh();
    if (!mesh) return false;

    matrix4 transform = node->getAbsoluteTransformation();
    verts.clear();
    tris.clear();

    irr::u32 vertexOffset = 0;

    for (irr::u32 i = 0; i < mesh->getMeshBufferCount(); ++i)
    {
        irr::scene::IMeshBuffer* buffer = mesh->getMeshBuffer(i);
        irr::u32 currentVertCount = buffer->getVertexCount();
        irr::u32 currentIdxCount = buffer->getIndexCount();

        if (currentVertCount == 0 || currentIdxCount == 0)
        {
            continue;
        }

        // Add vertices
        verts.reserve(verts.size() + currentVertCount * 3);
        for (irr::u32 j = 0; j < currentVertCount; ++j)
        {
            vector3df pos = buffer->getPosition(j);
            transform.transformVect(pos);
            verts.push_back(pos.X);
            verts.push_back(pos.Y);
            verts.push_back(pos.Z);
        }

        // Add triangles (indices)
        tris.reserve(tris.size() + currentIdxCount);
        const irr::u16* indices16 = buffer->getIndices();
        const irr::u32* indices32 = (const irr::u32*)indices16;

        if (buffer->getIndexType() == irr::video::EIT_16BIT)
        {
            for (irr::u32 j = 0; j < currentIdxCount; ++j)
            {
                tris.push_back(indices16[j] + vertexOffset);
            }
        }
        else // EIT_32BIT
        {
            for (irr::u32 j = 0; j < currentIdxCount; ++j)
            {
                tris.push_back(indices32[j] + vertexOffset);
            }
        }

        vertexOffset += currentVertCount;
    }

    if (verts.empty() || tris.empty())
    {
        printf("WARNING: StaticNavMesh::_getMeshBufferData: No vertices or triangles found.\n");
        return false;
    }

    printf("_getMeshBufferData: Total vertices=%u\n", vertexOffset);
    printf("_getMeshBufferData: Total indices=%zu\n", tris.size());
    return true;
}

bool CStaticNavMesh::_setupIrrSMeshFromRecastDetailMesh(irr::scene::SMesh* smesh)
{
    rcPolyMeshDetail* dmesh = _dmesh.get();
    if (!smesh || !dmesh)
        return false;

    irr::scene::SMeshBuffer* buffer = new irr::scene::SMeshBuffer();
    std::vector<float> vertsOut;
    int nvertsOut = 0;
    std::vector<int> trisOut;
    int ntrisOut = 0;

    if (!_getMeshDataFromPolyMeshDetail(dmesh, vertsOut, nvertsOut, trisOut, ntrisOut))
    {
        printf("ERROR: StaticNavMesh::_setupIrrSMeshFromRecastDetailMesh: _getMeshDataFromPolyMeshDetail failed.\n");
        buffer->drop();
        return false;
    }

    if (!_setMeshBufferData(*buffer, vertsOut, nvertsOut, trisOut, ntrisOut))
    {
        printf("ERROR: StaticNavMesh::_setupIrrSMeshFromRecastDetailMesh: _setMeshBufferData failed.\n");
        buffer->drop();
        return false;
    }

    smesh->addMeshBuffer(buffer);
    smesh->setHardwareMappingHint(irr::scene::EHM_STATIC, irr::scene::EBT_VERTEX_AND_INDEX);
    smesh->recalculateBoundingBox();
    buffer->drop();
    return true;
}

void CStaticNavMesh::_showHeightFieldInfo(const rcHeightfield& hf)
{
    printf("rcHeightfield hf: w=%i,h=%i,bmin=(%f,%f,%f),bmax=(%f,%f,%f),cs=%f,ch=%f\n",
        hf.width, hf.height, hf.bmin[0], hf.bmin[1], hf.bmin[2],
        hf.bmax[0], hf.bmax[1], hf.bmax[2], hf.cs, hf.ch);
}

bool CStaticNavMesh::_getMeshDataFromPolyMeshDetail(
    rcPolyMeshDetail* dmesh,
    std::vector<float>& vertsOut, int& nvertsOut,
    std::vector<int>& trisOut, int& ntrisOut)
{
    if (dmesh->nmeshes == 0)
    {
        printf("WARNING: StaticNavMesh::_getMeshDataFromPolyMeshDetail: dmesh->nmeshes == 0.\n");
        return false;
    }

    nvertsOut = dmesh->nverts * 3;
    ntrisOut = dmesh->ntris;

    vertsOut.resize(nvertsOut);
    memcpy(vertsOut.data(), dmesh->verts, nvertsOut * sizeof(float));

    trisOut.resize(ntrisOut * 3);
    int triIndex = 0;

    for (int p = 0; p < dmesh->nmeshes; ++p)
    {
        const unsigned int* m = &(dmesh->meshes[p * 4]);
        const unsigned int bverts = m[0];
        // const unsigned int nverts = m[1]; // Unused
        const unsigned int btris = m[2];
        const unsigned int ntris = m[3];
        const unsigned char* tris = &(dmesh->tris[btris * 4]);

        for (unsigned int n = 0; n < ntris; ++n)
        {
            // Correct logic: The triangle's vertex indices are relative to the detail mesh's
            // vertex list, but offset by `bverts` for this submesh.
            trisOut[triIndex++] = tris[n * 4 + 0] + bverts;
            trisOut[triIndex++] = tris[n * 4 + 1] + bverts;
            trisOut[triIndex++] = tris[n * 4 + 2] + bverts;
        }
    }

    return true;
}

bool CStaticNavMesh::_setMeshBufferData(
    irr::scene::SMeshBuffer& buffer,
    const std::vector<float>& verts, int& nverts,
    const std::vector<int>& tris, int& ntris)
{
    if (verts.empty() || tris.empty())
    {
        return false;
    }

    buffer.Vertices.set_used(nverts / 3);
    buffer.Indices.set_used(ntris * 3);

    for (int i = 0; i < nverts / 3; ++i)
    {
        buffer.Vertices[i] = irr::video::S3DVertex
        (
            verts[i * 3 + 0],
            verts[i * 3 + 1],
            verts[i * 3 + 2],
            0.0f, 1.0f, 0.0f, // Normal
            irr::video::SColor(255, 0, 255, 0),
            0.0f, 0.0f        // TCoords
        );
    }

    for (int i = 0; i < ntris * 3; ++i)
    {
        buffer.Indices[i] = tris[i];
    }

    buffer.getMaterial().Lighting = false;
    buffer.getMaterial().BackfaceCulling = false;
    buffer.getMaterial().Wireframe = true;
    buffer.getMaterial().Thickness = 2.0f;
    buffer.getMaterial().MaterialType = irr::video::EMT_SOLID;
    buffer.recalculateBoundingBox();

    return true;
}
