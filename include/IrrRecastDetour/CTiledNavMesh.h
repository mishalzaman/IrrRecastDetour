#pragma once

#include "INavMesh.h"
#include "Recast.h"
#include "DetourNavMeshBuilder.h"

/**
 * @class TiledNavMesh
 * @brief Implements AbstractNavMesh using the "tiled" (grid-based) build process.
 *
 * This class builds the navigation mesh in square chunks (tiles).
 * This is the preferred method for large worlds as it has a much lower
 * peak memory usage.
 *
 * It does not store most of the intermediate Recast data (heightfields, etc.)
 * as class members, because they are created and destroyed per-tile
 * inside the build() loop.
 */

namespace irr
{
    namespace scene
    {
        class CTiledNavMesh : public irr::scene::INavMesh
        {
        public:
            CTiledNavMesh(
                irr::scene::ISceneNode* parent,
                irr::scene::ISceneManager* mgr,
                irr::s32 id = -1
            );
            ~CTiledNavMesh();

            /**
             * @brief Builds a tiled navigation mesh from the given scene node.
             * @param levelNode The Irrlicht mesh node to use as geometry.
             * @param params The configuration parameters for the navmesh build.
             * @param tileSize The width/height of a single tile in grid units (not world units).
             * A good default is 32 or 64.
             * @return true if the build was successful, false otherwise.
             */
            bool build(
                irr::scene::IMeshSceneNode* levelNode,
                const NavMeshParams& params,
                const int tileSize
            );

            /**
             * @brief Creates (or re-creates) scene nodes visualizing the navmesh polygons.
             * @note This will create ONE debug node PER TILE.
             * @return A parent ISceneNode containing all tile debug meshes, or nullptr on failure.
             */
            irr::scene::ISceneNode* renderNavMesh();

            /**
             * @brief Gets the total time in milliseconds for the last successful build.
             */
            float getTotalBuildTimeMs() const { return _totalBuildTimeMs; }

        private:
            // --- Recast Build Context ---
            // (Other Recast objects are local to the build loop)
            std::unique_ptr<rcContext, RecastContextDeleter> _ctx;

            // --- Input Mesh Data (stored for the build loop) ---
            std::vector<float> _verts;
            std::vector<int> _tris;
            int _nverts;
            int _ntris;

            // --- Build Parameters ---
            NavMeshParams _params;
            int _tileSize;
            float _totalBuildTimeMs = 0.0f;

            // --- Debug Rendering ---
            // A parent node to hold all the tile debug meshes
            irr::scene::ISceneNode* _naviDebugParent = nullptr;
            // We store the dmesh data for each tile if keepInterResults is true
            std::vector<std::unique_ptr<rcPolyMeshDetail, RecastPolyMeshDetailDeleter>> _tileDMeshes;

            // --- Build Helper Functions ---

            /**
             * @brief Extracts vertex and index data from an Irrlicht mesh node.
             * (Identical to the helper in StaticNavMesh)
             */
            bool _getMeshBufferData(irr::scene::IMeshSceneNode* node);

            /**
             * @brief Builds a single tile and adds it to the dtNavMesh.
             * @param tx Tile X index.
             * @param ty Tile Y index.
             * @param bmin World bounding box min.
             * @param bmax World bounding box max.
             * @return The navmesh data for this tile, or nullptr on failure.
             */
            unsigned char* _buildTile(
                const int tx, const int ty,
                const float* bmin, const float* bmax,
                int& dataSize
            );

            // (Helper functions for creating Irrlicht debug geometry)
            bool _setupIrrSMeshFromRecastDetailMesh(irr::scene::SMesh* smesh, rcPolyMeshDetail* dmesh, irr::video::SColor color);
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
                const std::vector<int>& tris, int& ntris,
                irr::video::SColor color
            );
        };
	}
}
