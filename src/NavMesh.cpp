#include "NavMesh.h"
#include <irrlicht.h> // For Irrlicht types

// Use explicit namespaces
using irr::core::vector3df;
using irr::core::matrix4;
using irr::scene::ISceneNode;
using irr::scene::IMeshSceneNode;

NavMesh::NavMesh(irr::scene::ISceneNode* parent, irr::scene::ISceneManager* mgr, irr::s32 id)
    : irr::scene::ISceneNode(parent, mgr, id), // Call the base constructor
    _ctx(new rcContext()),
    _totalBuildTimeMs(0.0f)
{
    // This node is invisible by default
    setVisible(true);
}

NavMesh::~NavMesh()
{
}

void NavMesh::OnRegisterSceneNode()
{
    // Always register children, even if parent is invisible
    ISceneNode::OnRegisterSceneNode();

    // Only register self if visible (which it isn't, but children still render)
    if (IsVisible)
        SceneManager->registerNodeForRendering(this);
}

void NavMesh::render()
{
}

const irr::core::aabbox3d<irr::f32>& NavMesh::getBoundingBox() const
{
    return _box;
}

bool NavMesh::build(IMeshSceneNode* levelNode, const NavMeshParams& params)
{
    if (!levelNode)
    {
        printf("ERROR: NavMesh::build: levelNode is null.\n");
        return false;
    }

    _params = params;
    _totalBuildTimeMs = 0.0f;

    // Clear all previous build data
    // The .reset() calls will trigger the custom deleters on any existing data.
    _solid.reset();
    _chf.reset();
    _cset.reset();
    _pmesh.reset();
    _dmesh.reset();
    _navMesh.reset();
    _navQuery.reset();

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
        printf("ERROR: NavMesh::build: _getMeshBufferData() failed.\n");
        return false;
    }

    int nverts = _verts.size() / 3;
    int ntris = _tris.size() / 3;

    float bmin[3], bmax[3];
    rcCalcBounds(_verts.data(), nverts, bmin, bmax);

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

        for (int i = 0; i < _pmesh->npolys; ++i)
        {
            if (_pmesh->areas[i] == RC_WALKABLE_AREA)
                _pmesh->areas[i] = (unsigned char)SamplePolyAreas::GROUND;

            if (_pmesh->areas[i] == (unsigned char)SamplePolyAreas::GROUND ||
                _pmesh->areas[i] == (unsigned char)SamplePolyAreas::GRASS ||
                _pmesh->areas[i] == (unsigned char)SamplePolyAreas::ROAD)
            {
                _pmesh->flags[i] = (unsigned short)SamplePolyFlags::WALK;
            }
            else if (_pmesh->areas[i] == (unsigned char)SamplePolyAreas::WATER)
            {
                _pmesh->flags[i] = (unsigned short)SamplePolyFlags::SWIM;
            }
            else if (_pmesh->areas[i] == (unsigned char)SamplePolyAreas::DOOR)
            {
                _pmesh->flags[i] = (unsigned short)SamplePolyFlags::WALK | (unsigned short)SamplePolyFlags::DOOR;
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

        _navMesh.reset(dtAllocNavMesh());
        if (!_navMesh)
        {
            dtFree(navData);
            _ctx->log(RC_LOG_ERROR, "Could not create Detour navmesh");
            return false;
        }

        if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
        {
            _ctx->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
            return false;
        }

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

        // --- (NAVQUERY IS STILL NEEDED) ---
        _navQuery.reset(dtAllocNavMeshQuery());
        status = _navQuery->init(_navMesh.get(), 2048);
        if (dtStatusFailed(status))
        {
            _ctx->log(RC_LOG_ERROR, "Could not init Detour navmesh query");
            return false;
        }

        // --- (NEW) INITIALIZE CROWD ---
        _crowd.reset(dtAllocCrowd());
        if (!_crowd)
        {
            _ctx->log(RC_LOG_ERROR, "Could not alloc crowd");
            return false;
        }

        // Init the crowd with max agents, agent radius, and the navmesh
        // Note: The agent radius here is a 'max' radius. We'll set specifics in addAgent.
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

std::vector<vector3df> NavMesh::findPath(vector3df pStart, vector3df pEnd)
{
    std::vector<vector3df> lstPoints;
    if (!_navQuery || !_navMesh)
    {
        return lstPoints; // Not initialized
    }

    dtQueryFilter filter;
    dtPolyRef startRef, endRef;
    const int MAX_POLYS = 256;
    dtPolyRef returnedPath[MAX_POLYS];
    float straightPath[MAX_POLYS * 3];
    int numStraightPaths = 0;
    int pathCount = 0;

    float spos[3] = { pStart.X, pStart.Y, pStart.Z };
    float epos[3] = { pEnd.X, pEnd.Y, pEnd.Z };
    float polyPickExt[3] = { 2.0f, 4.0f, 2.0f };

    _navQuery->findNearestPoly(spos, polyPickExt, &filter, &startRef, nullptr);
    if (startRef == 0)
    {
        printf("WARNING: NavMesh::findPath: Could not find start polygon.\n");
        return lstPoints;
    }

    _navQuery->findNearestPoly(epos, polyPickExt, &filter, &endRef, nullptr);
    if (endRef == 0)
    {
        printf("WARNING: NavMesh::findPath: Could not find end polygon.\n");
        return lstPoints;
    }

    dtStatus findStatus = _navQuery->findPath(startRef, endRef, spos, epos, &filter, returnedPath, &pathCount, MAX_POLYS);

    if (dtStatusFailed(findStatus) || pathCount == 0)
    {
        printf("WARNING: NavMesh::findPath: Could not find path.\n");
        return lstPoints;
    }

    findStatus = _navQuery->findStraightPath(spos, epos, returnedPath,
        pathCount, straightPath, nullptr, nullptr, &numStraightPaths, MAX_POLYS);

    if (dtStatusFailed(findStatus))
    {
        printf("WARNING: NavMesh::findPath: Could not find straight path.\n");
        return lstPoints;
    }

    for (int i = 0; i < numStraightPaths; ++i)
    {
        vector3df cpos(straightPath[i * 3], straightPath[i * 3 + 1] + 0.25, // Small offset
            straightPath[i * 3 + 2]);
        lstPoints.push_back(cpos);
    }

    return lstPoints;
}

void NavMesh::renderDebugPath(const std::vector<vector3df>& path, irr::video::IVideoDriver* driver)
{
    if (path.size() < 2)
    {
        return;
    }

    irr::video::SMaterial m;
    m.Lighting = false;
    driver->setMaterial(m);
    driver->setTransform(irr::video::ETS_WORLD, matrix4());

    for (size_t i = 0; i < path.size() - 1; ++i)
    {
        driver->draw3DLine(path[i], path[i + 1], irr::video::SColor(255, 255, 0, 0));
    }
}

ISceneNode* NavMesh::createDebugMeshNode()
{
    if (!SceneManager)
    {
        printf("ERROR: NavMesh::createDebugMeshNode: SceneManager is null.\n");
        return nullptr;
    }
    if (!_dmesh)
    {
        printf("WARNING: NavMesh::createDebugMeshNode: No detail mesh data to build from.\n");
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
        printf("ERROR: NavMesh::createDebugMeshNode: _setupIrrSMeshFromRecastDetailMesh failed.\n");
        smesh->drop();
        return nullptr;
    }

    _naviDebugData = SceneManager->addMeshSceneNode(smesh, this); // Use regular mesh node instead of octree

    if (_naviDebugData)
    {
        _naviDebugData->setName("NavMeshDebugNode");
        _naviDebugData->setMaterialFlag(irr::video::EMF_LIGHTING, false);
        _naviDebugData->setMaterialFlag(irr::video::EMF_WIREFRAME, true);
        _naviDebugData->setPosition(irr::core::vector3df(0, 0.1f, 0));
        _naviDebugData->setVisible(true); // Explicitly set visible

        printf("Debug mesh created with %d vertices\n", smesh->getMeshBuffer(0)->getVertexCount());
    }

    smesh->drop();
    return _naviDebugData;
}

int NavMesh::addAgent(irr::scene::ISceneNode* node, float radius, float height)
{
    if (!_crowd || !node)
        return -1;

    dtCrowdAgentParams ap;
    memset(&ap, 0, sizeof(ap));
    ap.radius = radius;
    ap.height = height;
    ap.maxAcceleration = 20.0f;
    ap.maxSpeed = 3.5f; // You can make this a parameter
    ap.collisionQueryRange = ap.radius * 12.0f;
    ap.pathOptimizationRange = ap.radius * 30.0f;
    ap.updateFlags = DT_CROWD_ANTICIPATE_TURNS | DT_CROWD_OPTIMIZE_VIS | DT_CROWD_OPTIMIZE_TOPO | DT_CROWD_OBSTACLE_AVOIDANCE;

    vector3df pos = node->getPosition();
    float irrPos[3] = { pos.X, pos.Y - 0.5f, pos.Z }; // Assume position is center, adjust to feet

    int id = _crowd->addAgent(irrPos, &ap);
    if (id != -1)
    {
        _agentNodeMap[id] = node;
    }
    return id;
}

void NavMesh::setAgentTarget(int agentId, irr::core::vector3df targetPos)
{
    if (!_crowd || !_navQuery || agentId == -1)
        return;

    // Find the nearest polygon to the target position
    float pos[3] = { targetPos.X, targetPos.Y, targetPos.Z };
    float ext[3] = { 2.0f, 4.0f, 2.0f }; // Search extent
    dtPolyRef targetRef;
    float nearestPt[3];
    dtQueryFilter filter;

    _navQuery->findNearestPoly(pos, ext, &filter, &targetRef, nearestPt);

    if (targetRef)
    {
        // Request the agent to move to the new target
        _crowd->requestMoveTarget(agentId, targetRef, nearestPt);
    }
    else
    {
        printf("WARNING: NavMesh::setAgentTarget: Could not find poly for target.\n");
    }
}

void NavMesh::update(float deltaTime)
{
    if (!_crowd)
        return;

    // Update the crowd simulation
    _crowd->update(deltaTime, nullptr);

    // Update all Irrlicht nodes based on their agent's new position
    for (auto const& [id, node] : _agentNodeMap)
    {
        const dtCrowdAgent* agent = _crowd->getAgent(id);
        if (!agent || !agent->active)
            continue;

        // Get agent's position (at their feet)
        const float* pos = agent->npos;

        // Update the scene node's position
        // We add the agent's radius to the Y pos to place the sphere's *center*
        // This assumes the sphere's origin is its center.
        node->setPosition(vector3df(pos[0], pos[1] + agent->params.radius, pos[2]));
    }
}

void NavMesh::renderCrowdDebug(irr::video::IVideoDriver* driver)
{
    if (!_crowd || !driver)
        return;

    // Setup material for drawing lines
    irr::video::SMaterial m;
    m.Lighting = false;
    driver->setMaterial(m);
    driver->setTransform(irr::video::ETS_WORLD, matrix4());

    // Iterate over all active agents
    for (int i = 0; i < _crowd->getAgentCount(); ++i)
    {
        const dtCrowdAgent* agent = _crowd->getAgent(i);
        if (!agent || !agent->active)
            continue;

        // --- THE FIX ---

        // Check the number of corners
        if (agent->ncorners < 2) // Need at least 2 points to draw a line
            continue;

        // Get the path corners from agent->cornerVerts
        const float* pathPoints = agent->cornerVerts;

        // Draw lines between the waypoints
        for (int j = 0; j < agent->ncorners - 1; ++j)
        {
            // Points are (x, y, z)
            vector3df start(
                pathPoints[j * 3],
                pathPoints[j * 3 + 1] + 0.5f, // <-- INCREASED OFFSET
                pathPoints[j * 3 + 2]
            );
            vector3df end(
                pathPoints[(j + 1) * 3],
                pathPoints[(j + 1) * 3 + 1] + 0.5f, // <-- INCREASED OFFSET
                pathPoints[(j + 1) * 3 + 2]
            );

            // Draw a yellow line for the path
            driver->draw3DLine(start, end, irr::video::SColor(255, 255, 0, 0));
        }
    }
}


bool NavMesh::_getMeshBufferData(
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
        if (buffer->getIndexType() == irr::video::EIT_16BIT)
        {
            irr::u16* indices = buffer->getIndices();
            for (irr::u32 j = 0; j < currentIdxCount; ++j)
            {
                tris.push_back(indices[j] + vertexOffset);
            }
        }
        else if (buffer->getIndexType() == irr::video::EIT_32BIT)
        {
            irr::u32* indices = (irr::u32*)buffer->getIndices();
            for (irr::u32 j = 0; j < currentIdxCount; ++j)
            {
                tris.push_back(indices[j] + vertexOffset);
            }
        }

        vertexOffset += currentVertCount;
    }

    if (verts.empty() || tris.empty())
    {
        printf("WARNING: NavMesh::_getMeshBufferData: No vertices or triangles found.\n");
        return false;
    }

    printf("_getMeshBufferData: Total vertices=%u\n", vertexOffset);
    printf("_getMeshBufferData: Total indices=%zu\n", tris.size());
    return true;
}

bool NavMesh::_setupIrrSMeshFromRecastDetailMesh(irr::scene::SMesh* smesh)
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
        printf("ERROR: NavMesh::_setupIrrSMeshFromRecastDetailMesh: _getMeshDataFromPolyMeshDetail failed.\n");
        buffer->drop();
        return false;
    }

    if (!_setMeshBufferData(*buffer, vertsOut, nvertsOut, trisOut, ntrisOut))
    {
        printf("ERROR: NavMesh::_setupIrrSMeshFromRecastDetailMesh: _setMeshBufferData failed.\n");
        buffer->drop();
        return false;
    }

    smesh->addMeshBuffer(buffer);
    smesh->setHardwareMappingHint(irr::scene::EHM_STATIC, irr::scene::EBT_VERTEX_AND_INDEX);
    smesh->recalculateBoundingBox();
    buffer->drop();
    return true;
}

void NavMesh::_showHeightFieldInfo(const rcHeightfield& hf)
{
    printf("rcHeightfield hf: w=%i,h=%i,bmin=(%f,%f,%f),bmax=(%f,%f,%f),cs=%f,ch=%f\n",
        hf.width, hf.height, hf.bmin[0], hf.bmin[1], hf.bmin[2],
        hf.bmax[0], hf.bmax[1], hf.bmax[2], hf.cs, hf.ch);
}

bool NavMesh::_getMeshDataFromPolyMeshDetail(
    rcPolyMeshDetail* dmesh,
    std::vector<float>& vertsOut, int& nvertsOut,
    std::vector<int>& trisOut, int& ntrisOut)
{
    if (dmesh->nmeshes == 0)
    {
        printf("WARNING: NavMesh::_getMeshDataFromPolyMeshDetail: dmesh->nmeshes == 0.\n");
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

bool NavMesh::_setMeshBufferData(
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