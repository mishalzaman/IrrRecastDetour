#pragma once

#include <iostream>
#include <vector>
#include <stdio.h>
#include <cstddef>
#include <math.h>
#include <memory>
#include <map>

#include <irrlicht.h>
#include <ISceneNode.h>
// Only include Detour headers. Recast headers belong in the build-specific subclasses.
#include "DetourNavMeshQuery.h"
#include "DetourNavMesh.h"
#include "DetourCrowd.h"

// --- Custom Deleters for Detour objects ---
// (Recast-related deleters will go in the subclass that uses them)
struct DetourNavMeshDeleter {
    void operator()(dtNavMesh* navMesh) const { if (navMesh) dtFreeNavMesh(navMesh); }
};
struct DetourNavMeshQueryDeleter {
    void operator()(dtNavMeshQuery* navQuery) const { if (navQuery) dtFreeNavMeshQuery(navQuery); }
};
struct DetourCrowdDeleter {
    void operator()(dtCrowd* crowd) const { if (crowd) dtFreeCrowd(crowd); }
};

// --- Polygon Areas and Flags ---
// These are common definitions needed for agent query filters.
enum class SamplePolyAreas
{
    GROUND,
    WATER,
    ROAD,
    DOOR,
    GRASS,
    JUMP,
};
enum class SamplePolyFlags : unsigned short
{
    WALK = 0x01,       // Ability to walk (ground, grass, road)
    SWIM = 0x02,       // Ability to swim (water).
    DOOR = 0x04,       // Ability to move through doors.
    JUMP = 0x08,       // Ability to jump.
    DISABLED = 0x10,       // Disabled polygon
    ALL = 0xffff   // All abilities.
};

// Forward declaration for build parameters, which live in subclasses
struct NavMeshParams;


/**
 * @class AbstractNavMesh
 * @brief Base class for a Recast/Detour navigation mesh ISceneNode.
 *
 * This class manages the core Detour objects (dtNavMesh, dtNavMeshQuery, dtCrowd)
 * and provides all functionality for agent/crowd management and pathfinding queries.
 *
 * It is an abstract class because it does not implement the actual navmesh
 * *build* process. Subclasses (like StaticNavMesh or TiledNavMesh) must
 * implement their own build() method and are responsible for properly
 * initializing the protected _navMesh, _navQuery, and _crowd members.
 */
class AbstractNavMesh : public irr::scene::ISceneNode
{
public:
    AbstractNavMesh(
        irr::scene::ISceneNode* parent,
        irr::scene::ISceneManager* mgr,
        irr::s32 id = -1
    );

    virtual ~AbstractNavMesh();

    // Disable copy and move
    AbstractNavMesh(const AbstractNavMesh&) = delete;
    AbstractNavMesh& operator=(const AbstractNavMesh&) = delete;

    // --- ISceneNode Overrides ---
    virtual void OnRegisterSceneNode() override;
    virtual void render() override;
    virtual const irr::core::aabbox3d<irr::f32>& getBoundingBox() const override;

    // --- Agent (Crowd) Management ---

    /**
     * @brief Adds a new agent to the crowd simulation.
     * @param node The Irrlicht scene node this agent will control.
     * @param radius The agent's radius.
     * @param height The agent's height.
     * @return The agent's ID, or -1 on failure.
     */
    int addAgent(irr::scene::ISceneNode* node, float radius, float height);

    /**
     * @brief Adds a new agent to the crowd simulation (Advanced version).
     * @param node The Irrlicht scene node this agent will control.
     * @param params The Detour crowd agent parameters.
     * Any 0-value fields will be filled with defaults
     * (using _defaultAgentRadius/Height stored in this class).
     * @return The agent's ID, or -1 on failure.
     */
    int addAgent(irr::scene::ISceneNode* node, const dtCrowdAgentParams& params);

    /**
     * @brief Sets a new movement target for an agent.
     * @param agentId The ID returned by addAgent.
     * @param targetPos The world-space destination.
     */
    void setAgentTarget(int agentId, irr::core::vector3df targetPos);

    /**
     * @brief Updates the crowd simulation. Call this ONCE per frame.
     * @param deltaTime Time elapsed since the last frame, in seconds.
     */
    void update(float deltaTime);

    /**
    * @brief Renders debug lines for all agent paths in the crowd.
    * @param driver The Irrlicht video driver.
    */
    void renderAgentPaths(irr::video::IVideoDriver* driver);

    // --- Pathfinding Queries ---

    /**
     * @brief Finds the closest valid point on the navmesh to the given position.
     * @param pos The world-space position to query.
     * @return The closest valid position on the navmesh. Returns original pos if query fails.
     */
    irr::core::vector3df getClosestPointOnNavmesh(const irr::core::vector3df& pos);

protected:
    // --- Core Detour Objects (RAII-managed) ---
    // Subclasses are responsible for creating and initializing these.
    std::unique_ptr<dtNavMesh, DetourNavMeshDeleter> _navMesh;
    std::unique_ptr<dtNavMeshQuery, DetourNavMeshQueryDeleter> _navQuery;
    std::unique_ptr<dtCrowd, DetourCrowdDeleter> _crowd;

    // --- Agent Management ---
    std::map<int, irr::scene::ISceneNode*> _agentNodeMap;
    const int MAX_AGENTS = 1024; // (From original NavMesh.h)

    // Default agent params, to be set by subclass during build()
    float _defaultAgentRadius;
    float _defaultAgentHeight;

    // --- ISceneNode Data ---
    irr::core::aabbox3d<irr::f32> _box;
};