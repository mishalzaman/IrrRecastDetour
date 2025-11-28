/*
irrRecastDetour
Created by: Mishal Zaman
Version: 0.7.19
*/

#pragma once


#include <iostream>
#include <vector>
#include <stdio.h>
#include <cstddef>
#include <math.h>
#include <memory>
#include <map>

// Irrlicht includes
#include <irrlicht.h>
#include <ISceneNode.h>

// Detour includes
#include "DetourNavMeshQuery.h"
#include "DetourNavMesh.h"
#include "DetourCrowd.h"

// Recast includes
#include "Recast.h"
#include "DetourNavMeshBuilder.h"

// --- Custom Deleters for Detour objects ---
struct DetourNavMeshDeleter {
    void operator()(dtNavMesh* navMesh) const { if (navMesh) dtFreeNavMesh(navMesh); }
};
struct DetourNavMeshQueryDeleter {
    void operator()(dtNavMeshQuery* navQuery) const { if (navQuery) dtFreeNavMeshQuery(navQuery); }
};
struct DetourCrowdDeleter {
    void operator()(dtCrowd* crowd) const { if (crowd) dtFreeCrowd(crowd); }
};

// --- Custom Deleters for Recast objects ---
struct RecastContextDeleter {
    void operator()(rcContext* ctx) const { delete ctx; }
};
struct RecastHeightfieldDeleter {
    void operator()(rcHeightfield* hf) const { if (hf) rcFreeHeightField(hf); }
};
struct RecastCompactHeightfieldDeleter {
    void operator()(rcCompactHeightfield* chf) const { if (chf) rcFreeCompactHeightfield(chf); }
};
struct RecastContourSetDeleter {
    void operator()(rcContourSet* cset) const { if (cset) rcFreeContourSet(cset); }
};
struct RecastPolyMeshDeleter {
    void operator()(rcPolyMesh* pmesh) const { if (pmesh) rcFreePolyMesh(pmesh); }
};
struct RecastPolyMeshDetailDeleter {
    void operator()(rcPolyMeshDetail* dmesh) const { if (dmesh) rcFreePolyMeshDetail(dmesh); }
};

// --- Polygon Areas and Flags ---
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
    WALK = 0x01,            // Ability to walk (ground, grass, road)
    SWIM = 0x02,            // Ability to swim (water).
    DOOR = 0x04,            // Ability to move through doors.
    JUMP = 0x08,            // Ability to jump.
    DISABLED = 0x10,        // Disabled polygon
    ALL = 0xffff            // All abilities.
};

// Forward declaration for build parameters, which live in subclasses
struct NavMeshParams
{
    // Cell size in world units
    float CellSize = 0.3f;
    // Cell height in world units
    float CellHeight = 0.2f;

    // Agent parameters
    float AgentHeight = 2.0f;
    float AgentRadius = 0.6f;
    float AgentMaxClimb = 0.9f;
    float AgentMaxSlope = 45.0f;

    // Region parameters
    float RegionMinSize = 8.0f;
    float RegionMergeSize = 20.0f;

    // Edge parameters
    float EdgeMaxLen = 12.0f;
    float EdgeMaxError = 1.3f;

    // Polygon parameters
    float VertsPerPoly = 6.0f;

    // Detail mesh parameters
    float DetailSampleDist = 6.0f;
    float DetailSampleMaxError = 1.0f;

    // Partitioning method
    bool MonotonePartitioning = false;

    // Keep intermediate results for debug rendering
    bool KeepInterResults = false;
};


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
namespace irr {
    namespace scene {
        class INavMesh : public irr::scene::ISceneNode
        {
        public:
            INavMesh(
                irr::scene::ISceneNode* parent,
                irr::scene::ISceneManager* mgr,
                irr::s32 id = -1
            );

            virtual ~INavMesh();

            // Disable copy and move
            INavMesh(const INavMesh&) = delete;
            INavMesh& operator=(const INavMesh&) = delete;

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

            /**
             * @brief Gets a path between two positions on the navmesh.
             * @param startPos The starting world-space position.
             * @param endPos The ending world-space position.
             * @return A vector of waypoints along the path. Empty if no path found.
             */
            std::vector<irr::core::vector3df> GetPath(
                const irr::core::vector3df& startPos,
                const irr::core::vector3df& endPos
            );

            /**
             * @brief Calculates the distance along the navmesh between two positions.
             * @param startPos The starting world-space position.
             * @param endPos The ending world-space position.
             * @return The total distance along the path in world units. Returns -1.0f if no path found.
             */
            float GetPathDistance(
                const irr::core::vector3df& startPos,
                const irr::core::vector3df& endPos
            );

            /**
             * @brief Removes an agent from the crowd simulation.
             * @param agentId The ID of the agent to remove (returned by addAgent).
             * @return true if the agent was successfully removed, false otherwise.
             */
            void RemoveAgent(int agentId);

            /**
             * @brief Gets the current velocity of an agent.
             * @param agentId The ID of the agent.
             * @return The agent's velocity vector. Returns zero vector if agent not found.
             */
            irr::core::vector3df GetAgentVelocity(int agentId);

            /**
             * @brief Gets the current target position of an agent.
             * @param agentId The ID of the agent.
             * @return The agent's target position. Returns zero vector if agent not found or has no target.
             */
            irr::core::vector3df GetAgentCurrentTarget(int agentId);

            /**
             * @brief Checks if an agent has reached its destination.
             * @param agentId The ID of the agent.
             * @return true if the agent has reached its destination, false otherwise.
             */
            bool HasAgentReachedDestination(int agentId);

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
    }
}
