#include "IrrRecastDetour/INavMesh.h"
#include <irrlicht.h>

// Use explicit namespaces from original file
using irr::core::vector3df;
using irr::core::matrix4;
using irr::scene::ISceneNode;

using namespace irr;
using namespace scene;

INavMesh::INavMesh(irr::scene::ISceneNode* parent, irr::scene::ISceneManager* mgr, irr::s32 id)
    : irr::scene::ISceneNode(parent, mgr, id),
    _defaultAgentRadius(0.2f), // Set some sane defaults
    _defaultAgentHeight(1.0f)
{
    // The base node itself does not render, so it can be invisible.
    // Children (like debug meshes) can be made visible by the subclass.
    setVisible(false);
}

INavMesh::~INavMesh()
{
    // Smart pointers handle all cleanup automatically
}

void INavMesh::OnRegisterSceneNode()
{
    // Register children (like debug nodes added by subclasses)
    ISceneNode::OnRegisterSceneNode();

    // The abstract node itself doesn't render, so no need to register it.
}

void INavMesh::OnAnimate(irr::u32 timeMs)
{
    if (!_crowd)
        return;

    // --- 1. Calculate Delta Time ---
    // If this is the first call, initialize and skip the update
    if (_lastUpdateTimeMs == 0)
    {
        _lastUpdateTimeMs = timeMs;
        return;
    }

    // Delta time in seconds
    const float deltaTime = (timeMs - _lastUpdateTimeMs) / 1000.0f;
    _lastUpdateTimeMs = timeMs; // Update for next frame

    if (deltaTime == 0.0f)
        return; // Skip if no time has passed

    // --- 2. Update the Crowd Simulation ---
    // The second parameter (update_request) can usually be nullptr for simple updates
    _crowd->update(deltaTime, nullptr);

    // --- 3. Update all Irrlicht nodes based on their agent's new position ---
    for (auto const& [id, node] : _agentNodeMap)
    {
        const dtCrowdAgent* agent = _crowd->getAgent(id);
        if (!agent || !agent->active)
            continue;

        // Get agent's position (at their feet)
        const float* pos = agent->npos;

        // Update the scene node's position
        // This offset (pos[1] + height/2.0f) assumes the Irrlicht node's origin 
        // is at its center. This is a reasonable guess, but depends on your 
        // model/node setup.
        node->setPosition(irr::core::vector3df(
            pos[0], 
            pos[1] + (agent->params.height / 2.0f), 
            pos[2]
        ));
    }
}

void INavMesh::render()
{
    // This node does not render itself.
    // Agent path rendering is done from the main loop via renderAgentPaths().
    // Navmesh debug rendering is handled by the subclass.
}

const irr::core::aabbox3d<irr::f32>& INavMesh::getBoundingBox() const
{
    // This bounding box should be set by the subclass during its build()
    // to encompass the generated navigation mesh.
    return _box;
}

int INavMesh::addAgent(irr::scene::ISceneNode* node, float radius, float height)
{
    dtCrowdAgentParams ap;
    memset(&ap, 0, sizeof(ap)); // Zero out the struct

    // Set only the parameters we were given
    ap.radius = radius;
    ap.height = height;

    // Call the advanced function to handle the rest
    return this->addAgent(node, ap);
}

int INavMesh::addAgent(irr::scene::ISceneNode* node, const dtCrowdAgentParams& userParams)
{
    if (!_crowd || !node)
    {
        printf("ERROR: AbstractNavMesh::addAgent: Crowd or node is null.\n");
        return -1;
    }

    // Copy the user's params to a new struct that we can modify
    dtCrowdAgentParams finalParams = userParams;

    // --- Apply Defaults for "Missing" (0) Values ---

    // Set default radius/height from our stored defaults if not set
    if (finalParams.radius == 0.0f)
        finalParams.radius = _defaultAgentRadius;

    if (finalParams.height == 0.0f)
        finalParams.height = _defaultAgentHeight;

    // Set default movement params if not set
    if (finalParams.maxAcceleration == 0.0f)
        finalParams.maxAcceleration = 20.0f; // (From original file)

    if (finalParams.maxSpeed == 0.0f)
        finalParams.maxSpeed = 3.5f; // (From original file)

    // Set default query ranges if not set. These depend on the radius.
    if (finalParams.collisionQueryRange == 0.0f)
        finalParams.collisionQueryRange = finalParams.radius * 12.0f;

    if (finalParams.pathOptimizationRange == 0.0f)
        finalParams.pathOptimizationRange = finalParams.radius * 30.0f;

    // Set default update flags if not set
    if (finalParams.updateFlags == 0)
        finalParams.updateFlags = DT_CROWD_ANTICIPATE_TURNS | DT_CROWD_OPTIMIZE_VIS | DT_CROWD_OPTIMIZE_TOPO | DT_CROWD_OBSTACLE_AVOIDANCE;

    // --- Add Agent to Crowd ---

    vector3df pos = node->getPosition();

    // Agent position is at their feet.
    // We assume the node's position is its visual center.
    // So, we offset the Y position down by half the agent's height.
    float irrPos[3] = { pos.X, pos.Y - (finalParams.height / 2.0f), pos.Z };

    int id = _crowd->addAgent(irrPos, &finalParams);
    if (id != -1)
    {
        _agentNodeMap[id] = node;
    }
    else
    {
        printf("ERROR: AbstractNavMesh::addAgent: dtCrowd::addAgent failed.\n");
    }
    return id;
}

void INavMesh::setAgentTarget(int agentId, irr::core::vector3df targetPos)
{
    if (!_crowd || !_navQuery || agentId == -1)
    {
        if (!_crowd) printf("ERROR: setAgentTarget: No crowd.\n");
        if (!_navQuery) printf("ERROR: setAgentTarget: No navQuery.\n");
        return;
    }


    // Find the nearest polygon to the target position
    float pos[3] = { targetPos.X, targetPos.Y, targetPos.Z };
    float ext[3] = { 2.0f, 4.0f, 2.0f }; // Search extent
    dtPolyRef targetRef;
    float nearestPt[3];

    // Setup filter - allow walking on GROUND, ROAD, GRASS, and through DOORS
    dtQueryFilter filter;
    filter.setIncludeFlags((unsigned short)SamplePolyFlags::WALK | (unsigned short)SamplePolyFlags::DOOR);
    filter.setExcludeFlags(0);

    _navQuery->findNearestPoly(pos, ext, &filter, &targetRef, nearestPt);

    if (targetRef)
    {
        // Request the agent to move to the new target
        _crowd->requestMoveTarget(agentId, targetRef, nearestPt);
    }
    else
    {
        printf("WARNING: AbstractNavMesh::setAgentTarget: Could not find poly for target at (%f, %f, %f).\n", pos[0], pos[1], pos[2]);
    }
}

void INavMesh::renderAgentPaths(irr::video::IVideoDriver* driver)
{
    if (!_crowd || !driver)
        return;

    // Setup material for drawing lines
    irr::video::SMaterial m;
    m.Lighting = false;
    m.Thickness = 2.0f;
    driver->setMaterial(m);
    driver->setTransform(irr::video::ETS_WORLD, matrix4());

    // Iterate over all active agents
    for (int i = 0; i < _crowd->getAgentCount(); ++i)
    {
        const dtCrowdAgent* agent = _crowd->getAgent(i);

        // Skip if agent isn't active or has no path (no corners)
        if (!agent || !agent->active || agent->ncorners == 0)
            continue;

        // The path starts at the agent's current position.
        const float* p = agent->npos;
        vector3df startPoint(p[0], p[1] + 0.5f, p[2]); // Add a small Y-offset for visibility

        // Get the list of waypoints
        const float* pathPoints = agent->cornerVerts;

        // Iterate through all corners
        // The path is: agent->npos -> corner[0] -> corner[1] -> ...
        for (int j = 0; j < agent->ncorners; ++j)
        {
            // Get the current corner as the 'end' point for this segment
            vector3df endPoint(
                pathPoints[j * 3],
                pathPoints[j * 3 + 1] + 0.5f, // Add same Y-offset
                pathPoints[j * 3 + 2]
            );

            // Draw the line segment
            driver->draw3DLine(startPoint, endPoint, irr::video::SColor(255, 255, 0, 0)); // Red

            // For the next loop, the 'start' point becomes this 'end' point
            startPoint = endPoint;
        }
    }
}

irr::core::vector3df INavMesh::getClosestPointOnNavmesh(const irr::core::vector3df& pos)
{
    // If navmesh query isn't ready, return original position
    if (!_navQuery || !_navMesh)
    {
        return pos;
    }

    // Convert Irrlicht position to Detour format (float array)
    float queryPos[3] = { pos.X, pos.Y, pos.Z };

    // Search extents - how far to search for nearest polygon
    float extents[3] = { 2.0f, 4.0f, 2.0f }; // X, Y, Z search radius

    // Query filter - which polygon types we can walk on
    dtQueryFilter filter;
    filter.setIncludeFlags((unsigned short)SamplePolyFlags::WALK | (unsigned short)SamplePolyFlags::DOOR);
    filter.setExcludeFlags(0);

    // Output variables
    dtPolyRef nearestPoly = 0;
    float nearestPoint[3] = { 0, 0, 0 };

    // Find the nearest polygon and point on the navmesh
    dtStatus status = _navQuery->findNearestPoly(
        queryPos,
        extents,
        &filter,
        &nearestPoly,
        nearestPoint
    );

    // If successful and a polygon was found, return the clamped point
    if (dtStatusSucceed(status) && nearestPoly != 0)
    {
        return irr::core::vector3df(
            nearestPoint[0],
            nearestPoint[1],
            nearestPoint[2]
        );
    }

    // If query failed, return original position
    return pos;
}

std::vector<irr::core::vector3df> irr::scene::INavMesh::GetPath(const irr::core::vector3df& startPos, const irr::core::vector3df& endPos)
{
    std::vector<irr::core::vector3df> path;

    if (!_navQuery || !_navMesh)
    {
        printf("ERROR: INavMesh::getPath: NavQuery or NavMesh is null.\n");
        return path;
    }

    // Convert positions to Detour format
    float start[3] = { startPos.X, startPos.Y, startPos.Z };
    float end[3] = { endPos.X, endPos.Y, endPos.Z };

    // Search extents for finding nearest polygons
    float extents[3] = { 2.0f, 4.0f, 2.0f };

    // Query filter - which polygon types we can walk on
    dtQueryFilter filter;
    filter.setIncludeFlags((unsigned short)SamplePolyFlags::WALK | (unsigned short)SamplePolyFlags::DOOR);
    filter.setExcludeFlags(0);

    // Find nearest polygons for start and end positions
    dtPolyRef startRef = 0;
    dtPolyRef endRef = 0;
    float startNearest[3];
    float endNearest[3];

    dtStatus status = _navQuery->findNearestPoly(start, extents, &filter, &startRef, startNearest);
    if (dtStatusFailed(status) || startRef == 0)
    {
        printf("ERROR: INavMesh::getPath: Could not find start polygon.\n");
        return path;
    }

    status = _navQuery->findNearestPoly(end, extents, &filter, &endRef, endNearest);
    if (dtStatusFailed(status) || endRef == 0)
    {
        printf("ERROR: INavMesh::getPath: Could not find end polygon.\n");
        return path;
    }

    // Find the path (as a series of polygon references)
    const int MAX_POLYS = 256;
    dtPolyRef polys[MAX_POLYS];
    int polyCount = 0;

    status = _navQuery->findPath(startRef, endRef, startNearest, endNearest, &filter, polys, &polyCount, MAX_POLYS);
    if (dtStatusFailed(status) || polyCount == 0)
    {
        printf("ERROR: INavMesh::getPath: Could not find path.\n");
        return path;
    }

    // Convert polygon path to actual waypoints using findStraightPath
    const int MAX_STRAIGHT_PATH = 256;
    float straightPath[MAX_STRAIGHT_PATH * 3];
    unsigned char straightPathFlags[MAX_STRAIGHT_PATH];
    dtPolyRef straightPathPolys[MAX_STRAIGHT_PATH];
    int straightPathCount = 0;

    status = _navQuery->findStraightPath(
        startNearest, endNearest,
        polys, polyCount,
        straightPath, straightPathFlags, straightPathPolys,
        &straightPathCount, MAX_STRAIGHT_PATH,
        DT_STRAIGHTPATH_AREA_CROSSINGS
    );

    if (dtStatusFailed(status) || straightPathCount == 0)
    {
        printf("ERROR: INavMesh::getPath: Could not create straight path.\n");
        return path;
    }

    // Convert straight path to Irrlicht vector format
    path.reserve(straightPathCount);
    for (int i = 0; i < straightPathCount; ++i)
    {
        path.push_back(irr::core::vector3df(
            straightPath[i * 3],
            straightPath[i * 3 + 1],
            straightPath[i * 3 + 2]
        ));
    }

    return path;
}

float irr::scene::INavMesh::GetPathDistance(const irr::core::vector3df& startPos, const irr::core::vector3df& endPos)
{
    if (!_navQuery || !_navMesh)
    {
        printf("ERROR: INavMesh::getPathDistance: NavQuery or NavMesh is null.\n");
        return -1.0f;
    }

    // Convert positions to Detour format
    float start[3] = { startPos.X, startPos.Y, startPos.Z };
    float end[3] = { endPos.X, endPos.Y, endPos.Z };

    // Search extents for finding nearest polygons
    float extents[3] = { 2.0f, 4.0f, 2.0f };

    // Query filter - which polygon types we can walk on
    dtQueryFilter filter;
    filter.setIncludeFlags((unsigned short)SamplePolyFlags::WALK | (unsigned short)SamplePolyFlags::DOOR);
    filter.setExcludeFlags(0);

    // Find nearest polygons for start and end positions
    dtPolyRef startRef = 0;
    dtPolyRef endRef = 0;
    float startNearest[3];
    float endNearest[3];

    dtStatus status = _navQuery->findNearestPoly(start, extents, &filter, &startRef, startNearest);
    if (dtStatusFailed(status) || startRef == 0)
    {
        printf("ERROR: INavMesh::getPathDistance: Could not find start polygon.\n");
        return -1.0f;
    }

    status = _navQuery->findNearestPoly(end, extents, &filter, &endRef, endNearest);
    if (dtStatusFailed(status) || endRef == 0)
    {
        printf("ERROR: INavMesh::getPathDistance: Could not find end polygon.\n");
        return -1.0f;
    }

    // Find the path (as a series of polygon references)
    const int MAX_POLYS = 256;
    dtPolyRef polys[MAX_POLYS];
    int polyCount = 0;

    status = _navQuery->findPath(startRef, endRef, startNearest, endNearest, &filter, polys, &polyCount, MAX_POLYS);
    if (dtStatusFailed(status) || polyCount == 0)
    {
        printf("ERROR: INavMesh::getPathDistance: Could not find path.\n");
        return -1.0f;
    }

    // Convert polygon path to actual waypoints using findStraightPath
    const int MAX_STRAIGHT_PATH = 256;
    float straightPath[MAX_STRAIGHT_PATH * 3];
    unsigned char straightPathFlags[MAX_STRAIGHT_PATH];
    dtPolyRef straightPathPolys[MAX_STRAIGHT_PATH];
    int straightPathCount = 0;

    status = _navQuery->findStraightPath(
        startNearest, endNearest,
        polys, polyCount,
        straightPath, straightPathFlags, straightPathPolys,
        &straightPathCount, MAX_STRAIGHT_PATH,
        DT_STRAIGHTPATH_AREA_CROSSINGS
    );

    if (dtStatusFailed(status) || straightPathCount == 0)
    {
        printf("ERROR: INavMesh::getPathDistance: Could not create straight path.\n");
        return -1.0f;
    }

    // Calculate total distance by summing distances between consecutive waypoints
    float totalDistance = 0.0f;
    for (int i = 0; i < straightPathCount - 1; ++i)
    {
        float dx = straightPath[(i + 1) * 3 + 0] - straightPath[i * 3 + 0];
        float dy = straightPath[(i + 1) * 3 + 1] - straightPath[i * 3 + 1];
        float dz = straightPath[(i + 1) * 3 + 2] - straightPath[i * 3 + 2];

        float segmentDistance = sqrtf(dx * dx + dy * dy + dz * dz);
        totalDistance += segmentDistance;
    }

    return totalDistance;
}

void irr::scene::INavMesh::RemoveAgent(int agentId)
{
    if (!_crowd)
    {
        printf("ERROR: INavMesh::removeAgent: Crowd is null.\n");
        return;
    }

    if (agentId < 0 || agentId >= MAX_AGENTS)
    {
        printf("ERROR: INavMesh::removeAgent: Invalid agent ID: %d\n", agentId);
        return;
    }

    // Check if agent exists in our map
    auto it = _agentNodeMap.find(agentId);
    if (it == _agentNodeMap.end())
    {
        printf("WARNING: INavMesh::removeAgent: Agent ID %d not found in map.\n", agentId);
        return;
    }

    // Remove from Detour crowd
    _crowd->removeAgent(agentId);

    // Remove from our tracking map
    _agentNodeMap.erase(it);
}

irr::core::vector3df irr::scene::INavMesh::GetAgentVelocity(int agentId)
{
    if (!_crowd)
    {
        printf("ERROR: INavMesh::getAgentVelocity: Crowd is null.\n");
        return irr::core::vector3df(0, 0, 0);
    }

    if (agentId < 0 || agentId >= MAX_AGENTS)
    {
        printf("ERROR: INavMesh::getAgentVelocity: Invalid agent ID: %d\n", agentId);
        return irr::core::vector3df(0, 0, 0);
    }

    const dtCrowdAgent* agent = _crowd->getAgent(agentId);
    if (!agent || !agent->active)
    {
        printf("WARNING: INavMesh::getAgentVelocity: Agent %d not found or inactive.\n", agentId);
        return irr::core::vector3df(0, 0, 0);
    }

    // Return the actual velocity (nvel) being applied to the agent
    return irr::core::vector3df(agent->nvel[0], agent->nvel[1], agent->nvel[2]);
}

irr::core::vector3df irr::scene::INavMesh::GetAgentCurrentTarget(int agentId)
{
    if (!_crowd)
    {
        printf("ERROR: INavMesh::getAgentCurrentTarget: Crowd is null.\n");
        return irr::core::vector3df(0, 0, 0);
    }

    if (agentId < 0 || agentId >= MAX_AGENTS)
    {
        printf("ERROR: INavMesh::getAgentCurrentTarget: Invalid agent ID: %d\n", agentId);
        return irr::core::vector3df(0, 0, 0);
    }

    const dtCrowdAgent* agent = _crowd->getAgent(agentId);
    if (!agent || !agent->active)
    {
        printf("WARNING: INavMesh::getAgentCurrentTarget: Agent %d not found or inactive.\n", agentId);
        return irr::core::vector3df(0, 0, 0);
    }

    // Check if agent has a target
    if (agent->targetState == DT_CROWDAGENT_TARGET_NONE ||
        agent->targetState == DT_CROWDAGENT_TARGET_FAILED)
    {
        return irr::core::vector3df(0, 0, 0);
    }

    // Return the target position
    return irr::core::vector3df(agent->targetPos[0], agent->targetPos[1], agent->targetPos[2]);
}

bool irr::scene::INavMesh::HasAgentReachedDestination(int agentId)
{
    if (!_crowd)
    {
        printf("ERROR: INavMesh::hasAgentReachedDestination: Crowd is null.\n");
        return false;
    }

    if (agentId < 0 || agentId >= MAX_AGENTS)
    {
        printf("ERROR: INavMesh::hasAgentReachedDestination: Invalid agent ID: %d\n", agentId);
        return false;
    }

    const dtCrowdAgent* agent = _crowd->getAgent(agentId);
    if (!agent || !agent->active)
    {
        printf("WARNING: INavMesh::hasAgentReachedDestination: Agent %d not found or inactive.\n", agentId);
        return false;
    }

    // Agent has reached destination if:
    // 1. Target state is DT_CROWDAGENT_TARGET_VALID and ncorners is 0 (no more path corners)
    // 2. Or target state is DT_CROWDAGENT_TARGET_REACHED
    if (agent->targetState == DT_CROWDAGENT_TARGET_VALID && agent->ncorners == 0)
    {
        return true;
    }

    // Note: DT_CROWDAGENT_TARGET_REACHED might not exist in all Detour versions
    // In that case, the above check (ncorners == 0) is sufficient

    return false;
}
