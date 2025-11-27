#pragma once

#include "INavMesh.h"
#include "Recast.h"
#include "DetourNavMeshBuilder.h"

/**
 * @class StaticNavMesh
 * @brief Implements AbstractNavMesh using the "solo" (single mesh) build process.
 *
 * This class contains all the original build logic from your NavMesh class.
 * It holds all the intermediate Recast build data (heightfields, polymeshes)
 * and implements a build() function that generates a single navmesh from static geometry.
 */
class CStaticNavMesh : public irr::scene::INavMesh
{
public:
    CStaticNavMesh(
        irr::scene::ISceneNode* parent,
        irr::scene::ISceneManager* mgr,
        irr::s32 id = -1
    );
    ~CStaticNavMesh();

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
     * @brief Creates (or re-creates) a scene node visualizing the navmesh polygons.
     * @return The debug scene node, or nullptr if the build failed or smgr is null.
     */
    irr::scene::ISceneNode* renderNavMesh();

private:
    // --- Recast Build Objects (RAII-managed) ---
    std::unique_ptr<rcContext, RecastContextDeleter> _ctx;
    rcConfig _cfg;
    std::unique_ptr<rcHeightfield, RecastHeightfieldDeleter> _solid;
    std::unique_ptr<rcCompactHeightfield, RecastCompactHeightfieldDeleter> _chf;
    std::unique_ptr<rcContourSet, RecastContourSetDeleter> _cset;
    std::unique_ptr<rcPolyMesh, RecastPolyMeshDeleter> _pmesh;
    std::unique_ptr<rcPolyMeshDetail, RecastPolyMeshDetailDeleter> _dmesh;

    // --- Input Mesh Data ---
    std::vector<float> _verts;
    std::vector<int> _tris;
    std::vector<unsigned char> _triareas;

    // --- Build Options and Metrics ---
    NavMeshParams _params;
    float _totalBuildTimeMs = 0.0f;

    // --- Debug Rendering ---
    irr::scene::ISceneNode* _naviDebugData = nullptr;

    // --- Build Helper Functions ---
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