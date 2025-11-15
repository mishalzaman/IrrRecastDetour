#include "IrrRecastDetour/AbstractNavMesh.h"
#include <irrlicht.h>

// Use explicit namespaces from original file
using irr::core::vector3df;
using irr::core::matrix4;
using irr::scene::ISceneNode;

AbstractNavMesh::AbstractNavMesh(irr::scene::ISceneNode* parent, irr::scene::ISceneManager* mgr, irr::s32 id)
    : irr::scene::ISceneNode(parent, mgr, id),
    _defaultAgentRadius(0.2f), // Set some sane defaults
    _defaultAgentHeight(1.0f)
{
    // The base node itself does not render, so it can be invisible.
    // Children (like debug meshes) can be made visible by the subclass.
    setVisible(false);
}

AbstractNavMesh::~AbstractNavMesh()
{
    // Smart pointers handle all cleanup automatically
}

void AbstractNavMesh::OnRegisterSceneNode()
{
    // Register children (like debug nodes added by subclasses)
    ISceneNode::OnRegisterSceneNode();

    // The abstract node itself doesn't render, so no need to register it.
}

void AbstractNavMesh::render()
{
    // This node does not render itself.
    // Agent path rendering is done from the main loop via renderAgentPaths().
    // Navmesh debug rendering is handled by the subclass.
}

const irr::core::aabbox3d<irr::f32>& AbstractNavMesh::getBoundingBox() const
{
    // This bounding box should be set by the subclass during its build()
    // to encompass the generated navigation mesh.
    return _box;
}

int AbstractNavMesh::addAgent(irr::scene::ISceneNode* node, float radius, float height)
{
    dtCrowdAgentParams ap;
    memset(&ap, 0, sizeof(ap)); // Zero out the struct

    // Set only the parameters we were given
    ap.radius = radius;
    ap.height = height;

    // Call the advanced function to handle the rest
    return this->addAgent(node, ap);
}

int AbstractNavMesh::addAgent(irr::scene::ISceneNode* node, const dtCrowdAgentParams& userParams)
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

void AbstractNavMesh::setAgentTarget(int agentId, irr::core::vector3df targetPos)
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

void AbstractNavMesh::update(float deltaTime)
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
        // We add half the agent's height to place the node's origin at its visual center.
        node->setPosition(vector3df(pos[0], pos[1] + (agent->params.height / 2.0f), pos[2]));
    }
}

void AbstractNavMesh::renderAgentPaths(irr::video::IVideoDriver* driver)
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

irr::core::vector3df AbstractNavMesh::getClosestPointOnNavmesh(const irr::core::vector3df& pos)
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