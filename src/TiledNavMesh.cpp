#include "IrrRecastDetour/TiledNavMesh.h"
#include <irrlicht.h>

// Use explicit namespaces
using irr::core::vector3df;
using irr::core::matrix4;
using irr::scene::ISceneNode;
using irr::scene::IMeshSceneNode;

TiledNavMesh::TiledNavMesh(irr::scene::ISceneNode* parent, irr::scene::ISceneManager* mgr, irr::s32 id)
    : AbstractNavMesh(parent, mgr, id), // Call the base constructor
    _ctx(new rcContext(true)), // true = enable logging
    _nverts(0),
    _ntris(0),
    _tileSize(0)
{
}

TiledNavMesh::~TiledNavMesh()
{
    // All smart pointers handle their own cleanup.
    if (_naviDebugParent)
    {
        _naviDebugParent->remove();
    }
}

bool TiledNavMesh::build(IMeshSceneNode* levelNode, const NavMeshParams& params, const int tileSize)
{
    if (!levelNode)
    {
        printf("ERROR: TiledNavMesh::build: levelNode is null.\n");
        return false;
    }

    _params = params;
    _tileSize = tileSize;
    _totalBuildTimeMs = 0.0f;

    // Store defaults for the base class
    _defaultAgentRadius = _params.AgentRadius;
    _defaultAgentHeight = _params.AgentHeight;

    // Clear previous build data
    _navMesh.reset();
    _navQuery.reset();
    _crowd.reset();
    _tileDMeshes.clear();
    if (_naviDebugParent)
    {
        _naviDebugParent->remove();
        _naviDebugParent = nullptr;
    }

    _ctx->resetTimers();
    _ctx->startTimer(RC_TIMER_TOTAL);

    //
    // Step 1. Extract Irrlicht geometry
    //
    if (!this->_getMeshBufferData(levelNode))
    {
        printf("ERROR: TiledNavMesh::build: _getMeshBufferData() failed.\n");
        return false;
    }

    if (_nverts == 0 || _ntris == 0)
    {
            printf("ERROR: TiledNavMesh::build: No geometry found in levelNode.\n");
            return false;
    }

    float bmin[3], bmax[3];
    rcCalcBounds(_verts.data(), _nverts, bmin, bmax);

    // Set the ISceneNode's bounding box (from base class)
    _box.reset(irr::core::vector3df(bmin[0], bmin[1], bmin[2]));
    _box.addInternalPoint(irr::core::vector3df(bmax[0], bmax[1], bmax[2]));

    //
    // Step 2. Initialize dtNavMesh parameters for tiling
    //
    _navMesh.reset(dtAllocNavMesh());
    if (!_navMesh)
    {
        _ctx->log(RC_LOG_ERROR, "build: Could not alloc navmesh.");
        return false;
    }

    dtNavMeshParams dtParams;
    memset(&dtParams, 0, sizeof(dtParams));
    rcVcopy(dtParams.orig, bmin);
    dtParams.tileWidth = _tileSize * _params.CellSize;
    dtParams.tileHeight = _tileSize * _params.CellSize;

    // Calculate max tiles
    int gridW = 0, gridH = 0;
    rcCalcGridSize(bmin, bmax, _params.CellSize, &gridW, &gridH);
    const int tilesW = (gridW + _tileSize - 1) / _tileSize;
    const int tilesH = (gridH + _tileSize - 1) / _tileSize;
    dtParams.maxTiles = tilesW * tilesH;

    // Set max polys per tile. (A 128x128 tile is common)
    // This is a guess; 2048 is often safe.
    dtParams.maxPolys = 1 << 12; // 4096

    if (dtStatusFailed(_navMesh->init(&dtParams)))
    {
        _ctx->log(RC_LOG_ERROR, "build: Could not init navmesh.");
        return false;
    }

    //
    // Step 3. Build each tile
    //
    _ctx->log(RC_LOG_PROGRESS, "Building %d x %d tiles...", tilesW, tilesH);

    for (int y = 0; y < tilesH; ++y)
    {
        for (int x = 0; x < tilesW; ++x)
        {
            int dataSize = 0;
            unsigned char* data = _buildTile(x, y, bmin, bmax, dataSize);

            if (data)
            {
                // Add the tile data to the navmesh
                if (dtStatusFailed(_navMesh->addTile(data, dataSize, DT_TILE_FREE_DATA, 0, nullptr)))
                {
                    _ctx->log(RC_LOG_ERROR, "Failed to add tile %d, %d.", x, y);
                    dtFree(data);
                }
            }
        }
    }

    //
    // Step 4. Initialize Query and Crowd (from base class)
    //
    _navQuery.reset(dtAllocNavMeshQuery());
    if (dtStatusFailed(_navQuery->init(_navMesh.get(), 2048)))
    {
        _ctx->log(RC_LOG_ERROR, "build: Could not init Detour navmesh query");
        return false;
    }

    _crowd.reset(dtAllocCrowd());
    if (!_crowd)
    {
        _ctx->log(RC_LOG_ERROR, "build: Could not alloc crowd");
        return false;
    }

    if (!_crowd->init(MAX_AGENTS, _params.AgentRadius, _navMesh.get()))
    {
        _ctx->log(RC_LOG_ERROR, "build: Could not init crowd");
        return false;
    }

    _ctx->stopTimer(RC_TIMER_TOTAL);
    _totalBuildTimeMs = _ctx->getAccumulatedTime(RC_TIMER_TOTAL) / 1000.0f;
    _ctx->log(RC_LOG_PROGRESS, ">> Tiled build complete in %.1f ms.", _totalBuildTimeMs);

    return true;
}


unsigned char* TiledNavMesh::_buildTile(
    const int tx, const int ty,
    const float* bmin, const float* bmax,
    int& dataSize)
{
    dataSize = 0;

    rcConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.cs = _params.CellSize;
    cfg.ch = _params.CellHeight;
    cfg.walkableSlopeAngle = _params.AgentMaxSlope;
    cfg.walkableHeight = (int)ceilf(_params.AgentHeight / cfg.ch);
    cfg.walkableClimb = (int)floorf(_params.AgentMaxClimb / cfg.ch);
    cfg.walkableRadius = (int)ceilf(_params.AgentRadius / cfg.cs);
    cfg.maxEdgeLen = (int)(_params.EdgeMaxLen / cfg.cs);
    cfg.maxSimplificationError = _params.EdgeMaxError;
    cfg.minRegionArea = (int)rcSqr(_params.RegionMinSize);
    cfg.mergeRegionArea = (int)rcSqr(_params.RegionMergeSize);
    cfg.maxVertsPerPoly = (int)_params.VertsPerPoly;
    cfg.detailSampleDist = _params.DetailSampleDist < 0.9f ? 0 : cfg.cs * _params.DetailSampleDist;
    cfg.detailSampleMaxError = cfg.ch * _params.DetailSampleMaxError;
    cfg.tileSize = _tileSize; // Set the tile size
    cfg.borderSize = cfg.walkableRadius + 3; // 3 = default padding
    cfg.width = cfg.tileSize + cfg.borderSize * 2;
    cfg.height = cfg.tileSize + cfg.borderSize * 2;

    // Calculate tile bounds
    cfg.bmin[0] = bmin[0] + tx * cfg.tileSize * cfg.cs;
    cfg.bmin[1] = bmin[1];
    cfg.bmin[2] = bmin[2] + ty * cfg.tileSize * cfg.cs;
    cfg.bmax[0] = bmin[0] + (tx + 1) * cfg.tileSize * cfg.cs;
    cfg.bmax[1] = bmax[1];
    cfg.bmax[2] = bmin[2] + (ty + 1) * cfg.tileSize * cfg.cs;

    // Expand bounds by border size
    cfg.bmin[0] -= cfg.borderSize * cfg.cs;
    cfg.bmin[2] -= cfg.borderSize * cfg.cs;
    cfg.bmax[0] += cfg.borderSize * cfg.cs;
    cfg.bmax[2] += cfg.borderSize * cfg.cs;

    _ctx->log(RC_LOG_PROGRESS, "Building tile %d, %d...", tx, ty);

    //
    // NOTE: All Recast objects are local to this function and auto-deleted
    // by smart pointers when the function exits.
    //

    // Step 3. Rasterize
    std::unique_ptr<rcHeightfield, RecastHeightfieldDeleter> solid(rcAllocHeightfield());
    if (!solid)
    {
        _ctx->log(RC_LOG_ERROR, "buildTile: Out of memory 'solid'.");
        return nullptr;
    }
    if (!rcCreateHeightfield(_ctx.get(), *solid, cfg.width, cfg.height, cfg.bmin, cfg.bmax, cfg.cs, cfg.ch))
    {
        _ctx->log(RC_LOG_ERROR, "buildTile: Could not create solid heightfield.");
        return nullptr;
    }

    std::vector<unsigned char> triareas(_ntris);
    memset(triareas.data(), 0, _ntris * sizeof(unsigned char));
    rcMarkWalkableTriangles(_ctx.get(), cfg.walkableSlopeAngle, _verts.data(), _nverts, _tris.data(), _ntris, triareas.data());
    // Rasterize *all* geometry. Recast will clip it to the heightfield's bounds.
    rcRasterizeTriangles(_ctx.get(), _verts.data(), _nverts, _tris.data(), triareas.data(), _ntris, *solid, cfg.walkableClimb);

    // Step 4. Filter
    rcFilterLowHangingWalkableObstacles(_ctx.get(), cfg.walkableClimb, *solid);
    rcFilterLedgeSpans(_ctx.get(), cfg.walkableHeight, cfg.walkableClimb, *solid);
    rcFilterWalkableLowHeightSpans(_ctx.get(), cfg.walkableHeight, *solid);

    // Step 5. Partition
    std::unique_ptr<rcCompactHeightfield, RecastCompactHeightfieldDeleter> chf(rcAllocCompactHeightfield());
    if (!chf)
    {
        _ctx->log(RC_LOG_ERROR, "buildTile: Out of memory 'chf'.");
        return nullptr;
    }
    if (!rcBuildCompactHeightfield(_ctx.get(), cfg.walkableHeight, cfg.walkableClimb, *solid, *chf))
    {
        _ctx->log(RC_LOG_ERROR, "buildTile: Could not build compact data.");
        return nullptr;
    }

    solid.reset(); // No longer needed

    if (!rcErodeWalkableArea(_ctx.get(), cfg.walkableRadius, *chf))
    {
        _ctx->log(RC_LOG_ERROR, "buildTile: Could not erode.");
        return nullptr;
    }

    if (_params.MonotonePartitioning)
    {
        if (!rcBuildRegionsMonotone(_ctx.get(), *chf, cfg.borderSize, cfg.minRegionArea, cfg.mergeRegionArea))
        {
            _ctx->log(RC_LOG_ERROR, "buildTile: Could not build regions.");
            return nullptr;
        }
    }
    else
    {
        if (!rcBuildDistanceField(_ctx.get(), *chf))
        {
            _ctx->log(RC_LOG_ERROR, "buildTile: Could not build distance field.");
            return nullptr;
        }
        if (!rcBuildRegions(_ctx.get(), *chf, cfg.borderSize, cfg.minRegionArea, cfg.mergeRegionArea))
        {
            _ctx->log(RC_LOG_ERROR, "buildTile: Could not build regions.");
            return nullptr;
        }
    }

    // Step 6. Contours
    std::unique_ptr<rcContourSet, RecastContourSetDeleter> cset(rcAllocContourSet());
    if (!cset)
    {
        _ctx->log(RC_LOG_ERROR, "buildTile: Out of memory 'cset'.");
        return nullptr;
    }
    if (!rcBuildContours(_ctx.get(), *chf, cfg.maxSimplificationError, cfg.maxEdgeLen, *cset))
    {
        _ctx->log(RC_LOG_ERROR, "buildTile: Could not create contours.");
        return nullptr;
    }

    // Step 7. PolyMesh
    std::unique_ptr<rcPolyMesh, RecastPolyMeshDeleter> pmesh(rcAllocPolyMesh());
    if (!pmesh)
    {
        _ctx->log(RC_LOG_ERROR, "buildTile: Out of memory 'pmesh'.");
        return nullptr;
    }
    if (!rcBuildPolyMesh(_ctx.get(), *cset, cfg.maxVertsPerPoly, *pmesh))
    {
        _ctx->log(RC_LOG_ERROR, "buildTile: Could not triangulate contours.");
        return nullptr;
    }

    cset.reset(); // No longer needed

    // Step 8. DetailMesh
    std::unique_ptr<rcPolyMeshDetail, RecastPolyMeshDetailDeleter> dmesh(rcAllocPolyMeshDetail());
    if (!dmesh)
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
        return nullptr;
    }
    if (!rcBuildPolyMeshDetail(_ctx.get(), *pmesh, *chf, cfg.detailSampleDist, cfg.detailSampleMaxError, *dmesh))
    {
        _ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
        return nullptr;
    }

    chf.reset(); // No longer needed

    // Step 9. Create Detour data
    if (cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
    {
        // Set polygon flags
        for (int i = 0; i < pmesh->npolys; ++i)
        {
            if (pmesh->areas[i] == RC_WALKABLE_AREA)
            {
                pmesh->areas[i] = (unsigned char)SamplePolyAreas::GROUND;
                pmesh->flags[i] = (unsigned short)SamplePolyFlags::WALK;
            }
        }

        dtNavMeshCreateParams dtParams;
        memset(&dtParams, 0, sizeof(dtParams));
        dtParams.verts = pmesh->verts;
        dtParams.vertCount = pmesh->nverts;
        dtParams.polys = pmesh->polys;
        dtParams.polyAreas = pmesh->areas;
        dtParams.polyFlags = pmesh->flags;
        dtParams.polyCount = pmesh->npolys;
        dtParams.nvp = pmesh->nvp;
        dtParams.detailMeshes = dmesh->meshes;
        dtParams.detailVerts = dmesh->verts;
        dtParams.detailVertsCount = dmesh->nverts;
        dtParams.detailTris = dmesh->tris;
        dtParams.detailTriCount = dmesh->ntris;

        dtParams.walkableHeight = _params.AgentHeight;
        dtParams.walkableRadius = _params.AgentRadius;
        dtParams.walkableClimb = _params.AgentMaxClimb;
        rcVcopy(dtParams.bmin, pmesh->bmin);
        rcVcopy(dtParams.bmax, pmesh->bmax);
        dtParams.cs = cfg.cs;
        dtParams.ch = cfg.ch;
        dtParams.tileX = tx;
        dtParams.tileY = ty;
        dtParams.buildBvTree = false; // Not needed for tiled mesh

        unsigned char* navData = nullptr;
        if (!dtCreateNavMeshData(&dtParams, &navData, &dataSize))
        {
            _ctx->log(RC_LOG_ERROR, "Could not build Detour navmesh tile data.");
            return nullptr;
        }

        // Store the detail mesh if we want to render it later
        if (_params.KeepInterResults)
        {
            _tileDMeshes.push_back(std::move(dmesh));
        }

        return navData;
    }

    _ctx->log(RC_LOG_ERROR, "maxVertsPerPoly > DT_VERTS_PER_POLYGON");
    return nullptr;
}

ISceneNode* TiledNavMesh::renderNavMesh()
{
    if (!SceneManager)
    {
        printf("ERROR: TiledNavMesh::renderNavMesh: SceneManager is null.\n");
        return nullptr;
    }
    if (!_params.KeepInterResults || _tileDMeshes.empty())
    {
        printf("WARNING: TiledNavMesh::renderNavMesh: No detail mesh data was kept.\n");
        printf("         Set KeepInterResults = true in NavMeshParams to debug render.\n");
        return nullptr;
    }

    // Remove old debug parent if it exists
    if (_naviDebugParent)
    {
        _naviDebugParent->remove();
        _naviDebugParent = nullptr;
    }

    // Create a new parent node
    _naviDebugParent = SceneManager->addEmptySceneNode(this);
    _naviDebugParent->setName("NavMeshDebugNode_Parent");


    int i = 0;
    for (const auto& dmesh : _tileDMeshes)
    {
        if (!dmesh) continue;

        irr::scene::SMesh* smesh = new irr::scene::SMesh();
        if (!_setupIrrSMeshFromRecastDetailMesh(smesh, dmesh.get()))
        {
            _ctx->log(RC_LOG_WARNING, "Failed to setup debug mesh for a tile.");
            smesh->drop();
            continue;
        }

        // Add the debug node as a child of the parent
        ISceneNode* tileNode = SceneManager->addMeshSceneNode(smesh, _naviDebugParent);

        if (tileNode)
        {
            char name[64];
            snprintf(name, 64, "NavMeshDebugTile_%d", i++);
            tileNode->setName(name);
            tileNode->setMaterialFlag(irr::video::EMF_LIGHTING, false);
            tileNode->setMaterialFlag(irr::video::EMF_WIREFRAME, true);
            tileNode->setPosition(irr::core::vector3df(0, 0, 0));
            tileNode->setVisible(true);
        }

        smesh->drop();
    }

    this->setVisible(true);

    return _naviDebugParent;
}


// --- Private Helper Functions (Copied from StaticNavMesh) ---

bool TiledNavMesh::_getMeshBufferData(IMeshSceneNode* node)
{
    if (!node) return false;
    irr::scene::IMesh* mesh = node->getMesh();
    if (!mesh) return false;

    matrix4 transform = node->getAbsoluteTransformation();
    _verts.clear();
    _tris.clear();

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

        _verts.reserve(_verts.size() + currentVertCount * 3);
        for (irr::u32 j = 0; j < currentVertCount; ++j)
        {
            vector3df pos = buffer->getPosition(j);
            transform.transformVect(pos);
            _verts.push_back(pos.X);
            _verts.push_back(pos.Y);
            _verts.push_back(pos.Z);
        }

        _tris.reserve(_tris.size() + currentIdxCount);
        const irr::u16* indices16 = buffer->getIndices();
        const irr::u32* indices32 = (const irr::u32*)indices16;

        if (buffer->getIndexType() == irr::video::EIT_16BIT)
        {
            for (irr::u32 j = 0; j < currentIdxCount; ++j)
            {
                _tris.push_back(indices16[j] + vertexOffset);
            }
        }
        else // EIT_32BIT
        {
            for (irr::u32 j = 0; j < currentIdxCount; ++j)
            {
                _tris.push_back(indices32[j] + vertexOffset);
            }
        }
        vertexOffset += currentVertCount;
    }

    if (_verts.empty() || _tris.empty())
    {
        printf("WARNING: TiledNavMesh::_getMeshBufferData: No vertices or triangles found.\n");
        return false;
    }

    _nverts = _verts.size() / 3;
    _ntris = _tris.size() / 3;

    printf("_getMeshBufferData: Total vertices=%d\n", _nverts);
    printf("_getMeshBufferData: Total triangles=%d\n", _ntris);
    return true;
}

bool TiledNavMesh::_setupIrrSMeshFromRecastDetailMesh(irr::scene::SMesh* smesh, rcPolyMeshDetail* dmesh)
{
    if (!smesh || !dmesh)
        return false;

    irr::scene::SMeshBuffer* buffer = new irr::scene::SMeshBuffer();
    std::vector<float> vertsOut;
    int nvertsOut = 0;
    std::vector<int> trisOut;
    int ntrisOut = 0;

    if (!_getMeshDataFromPolyMeshDetail(dmesh, vertsOut, nvertsOut, trisOut, ntrisOut))
    {
        buffer->drop();
        return false;
    }

    if (!_setMeshBufferData(*buffer, vertsOut, nvertsOut, trisOut, ntrisOut))
    {
        buffer->drop();
        return false;
    }

    smesh->addMeshBuffer(buffer);
    smesh->setHardwareMappingHint(irr::scene::EHM_STATIC, irr::scene::EBT_VERTEX_AND_INDEX);
    smesh->recalculateBoundingBox();
    buffer->drop();
    return true;
}

bool TiledNavMesh::_getMeshDataFromPolyMeshDetail(
    rcPolyMeshDetail* dmesh,
    std::vector<float>& vertsOut, int& nvertsOut,
    std::vector<int>& trisOut, int& ntrisOut)
{
    if (dmesh->nmeshes == 0)
    {
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
        const unsigned int btris = m[2];
        const unsigned int ntris = m[3];
        const unsigned char* tris = &(dmesh->tris[btris * 4]);

        for (unsigned int n = 0; n < ntris; ++n)
        {
            trisOut[triIndex++] = tris[n * 4 + 0] + bverts;
            trisOut[triIndex++] = tris[n * 4 + 1] + bverts;
            trisOut[triIndex++] = tris[n * 4 + 2] + bverts;
        }
    }

    return true;
}

bool TiledNavMesh::_setMeshBufferData(
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