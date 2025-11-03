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
#include "Recast.h"
#include "DetourNavMeshQuery.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourCrowd.h"


// Define custom deleters for Recast/Detour types
struct RecastContextDeleter {
    void operator()(rcContext* ctx) const { if (ctx) delete ctx; }
};
struct RecastHeightfieldDeleter {
    void operator()(rcHeightfield* hf) const { rcFreeHeightField(hf); }
};
struct RecastCompactHeightfieldDeleter {
    void operator()(rcCompactHeightfield* chf) const { rcFreeCompactHeightfield(chf); }
};
struct RecastContourSetDeleter {
    void operator()(rcContourSet* cset) const { rcFreeContourSet(cset); }
};
struct RecastPolyMeshDeleter {
    void operator()(rcPolyMesh* pmesh) const { rcFreePolyMesh(pmesh); }
};
struct RecastPolyMeshDetailDeleter {
    void operator()(rcPolyMeshDetail* dmesh) const { rcFreePolyMeshDetail(dmesh); }
};
struct DetourNavMeshDeleter {
    void operator()(dtNavMesh* navMesh) const { dtFreeNavMesh(navMesh); }
};
struct DetourNavMeshQueryDeleter {
    void operator()(dtNavMeshQuery* navQuery) const { dtFreeNavMeshQuery(navQuery); }
};

struct DetourCrowdDeleter {
    void operator()(dtCrowd* crowd) const { dtFreeCrowd(crowd); }
};

/// These are just sample areas to use consistent values across the samples.
/// The use should specify these base on his needs.
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

struct NavMeshParams
{
    float CellSize = 0.2f;
    float CellHeight = 0.2f;
    float AgentHeight = 1.0f;
    float AgentRadius = 0.2f;
    float AgentMaxClimb = 0.7f;
    float AgentMaxSlope = 85.0f;
    int   RegionMinSize = 8;
    int   RegionMergeSize = 20;
    bool  MonotonePartitioning = false;
    float EdgeMaxLen = 12.0f;
    float EdgeMaxError = 0.1f;
    float VertsPerPoly = 6.0f;
    float DetailSampleDist = 3.0f;
    float DetailSampleMaxError = 0.5f;
    bool  KeepInterResults = false;
};

class NavMesh : public irr::scene::ISceneNode
{
public:
    NavMesh(
        irr::scene::ISceneNode* parent,
        irr::scene::ISceneManager* mgr,
        irr::s32 id = -1
    );
    ~NavMesh();

    // Disable copy and move
    NavMesh(const NavMesh&) = delete;
    NavMesh& operator=(const NavMesh&) = delete;

    // Override methods from ISceneNode
    virtual void OnRegisterSceneNode() override;
    virtual void render() override;
    virtual const irr::core::aabbox3d<irr::f32>& getBoundingBox() const override;

    /**
     * @brief Builds the navigation mesh from the given scene node.
     * @param levelNode The Irrlicht mesh node to use as geometry.
     * @param params The configuration parameters for the navmesh build.
     * @return true if the build was successful, false otherwise.
     */
    bool build(
        irr::scene::IMeshSceneNode* levelNode,
        const NavMeshParams& params
    );

    /**
     * @brief Gets the total time in milliseconds for the last successful build.
     */
    float getTotalBuildTimeMs() const { return _totalBuildTimeMs; }

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
         * @param params The Detour crowd agent parameters. Any 0-value fields will be filled with defaults.
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
     * @brief Creates (or re-creates) a scene node visualizing the navmesh polygons.
     * @param smgr The Irrlicht scene manager.
     * @return The debug scene node, or nullptr if the build failed or smgr is null.
     */
    irr::scene::ISceneNode* renderNavMesh();

    /**
    * @brief Renders debug lines for all agent paths in the crowd.
    * @param driver The Irrlicht video driver.
    */
    void renderAgentPaths(irr::video::IVideoDriver* driver);

private:
    // Recast/Detour core objects (RAII-managed)
    std::unique_ptr<rcContext, RecastContextDeleter> _ctx;
    rcConfig _cfg;
    std::unique_ptr<rcHeightfield, RecastHeightfieldDeleter> _solid;
    std::unique_ptr<rcCompactHeightfield, RecastCompactHeightfieldDeleter> _chf;
    std::unique_ptr<rcContourSet, RecastContourSetDeleter> _cset;
    std::unique_ptr<rcPolyMesh, RecastPolyMeshDeleter> _pmesh;
    std::unique_ptr<rcPolyMeshDetail, RecastPolyMeshDetailDeleter> _dmesh;

    // Detour navigation query objects (RAII-managed)
    std::unique_ptr<dtNavMeshQuery, DetourNavMeshQueryDeleter> _navQuery;
    std::unique_ptr<dtNavMesh, DetourNavMeshDeleter> _navMesh;

    // Input mesh data
    std::vector<float> _verts;
    std::vector<int> _tris;
    std::vector<unsigned char> _triareas;

    // Build options and metrics
    NavMeshParams _params;
    float _totalBuildTimeMs;

    irr::scene::ISceneNode* _naviDebugData = nullptr;

    irr::core::aabbox3d<irr::f32> _box;

    std::unique_ptr<dtCrowd, DetourCrowdDeleter> _crowd;

    // Links dtCrowd agent IDs to Irrlicht scene nodes
    std::map<int, irr::scene::ISceneNode*> _agentNodeMap;

	const int MAX_AGENTS = 128;

    bool _getMeshBufferData
    (
        irr::scene::IMeshSceneNode* node,
        std::vector<float>& verts,
        std::vector<int>& tris
    );
    bool _setupIrrSMeshFromRecastDetailMesh(irr::scene::SMesh* smesh);
    void _showHeightFieldInfo(const rcHeightfield& hf);
    bool _getMeshDataFromPolyMeshDetail
    (
        rcPolyMeshDetail* dmesh,
        std::vector<float>& vertsOut, int& nvertsOut,
        std::vector<int>& trisOut, int& ntrisOut
    );
    bool _setMeshBufferData
    (
        irr::scene::SMeshBuffer& buffer,
        const std::vector<float>& verts, int& nverts,
        const std::vector<int>& tris, int& ntris
    );
};