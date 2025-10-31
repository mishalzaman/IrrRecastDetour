#pragma once

#include <iostream>
#include <vector>
#include <stdio.h>
#include <cstddef>
#include <math.h>
#include <irrlicht.h>
#include "Recast.h"
#include "DetourNavMeshQuery.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"

using namespace std;
using namespace irr;
using namespace irr::core;

/// These are just sample areas to use consistent values across the samples.
/// The use should specify these base on his needs.
enum SamplePolyAreas
{
    SAMPLE_POLYAREA_GROUND,
    SAMPLE_POLYAREA_WATER,
    SAMPLE_POLYAREA_ROAD,
    SAMPLE_POLYAREA_DOOR,
    SAMPLE_POLYAREA_GRASS,
    SAMPLE_POLYAREA_JUMP,
};
enum SamplePolyFlags
{
    SAMPLE_POLYFLAGS_WALK = 0x01,       // Ability to walk (ground, grass, road)
    SAMPLE_POLYFLAGS_SWIM = 0x02,       // Ability to swim (water).
    SAMPLE_POLYFLAGS_DOOR = 0x04,       // Ability to move through doors.
    SAMPLE_POLYFLAGS_JUMP = 0x08,       // Ability to jump.
    SAMPLE_POLYFLAGS_DISABLED = 0x10,       // Disabled polygon
    SAMPLE_POLYFLAGS_ALL = 0xffff   // All abilities.
};

class NavMesh
{
public:
    NavMesh();
    ~NavMesh();

    bool load(
        scene::IMeshSceneNode* levelNode,
        scene::ISceneManager* smgr
    );

    std::vector<irr::core::vector3df> getPath(
        core::vector3df start,
        core::vector3df end
    );

    void renderDebugPath(
        std::vector<irr::core::vector3df> path,
        video::IVideoDriver* driver
    );

private:
    // Recast/Detour core objects
    rcContext* _ctx;
    rcConfig _cfg;
    rcHeightfield* _solid;
    rcCompactHeightfield* _chf;
    rcContourSet* _cset;
    rcPolyMesh* _pmesh;
    rcPolyMeshDetail* _dmesh;

    // Detour navigation query objects
    dtNavMeshQuery* _navQuery;
    dtNavMesh* _navMesh;

    // Input mesh data
    float* _verts;
    int* _tris;
    unsigned char* _triareas;

    // Agent configuration
    float _agentHeight;
    float _agentRadius;
    float _agentMaxClimb;
    float _agentMaxSlope;

    // Voxelization parameters
    float _cellSize;
    float _cellHeight;

    // Region generation parameters
    float _regionMinSize;
    float _regionMergeSize;
    bool _monotonePartitioning;

    // Polygon mesh generation parameters
    float _edgeMaxLen;
    float _edgeMaxError;
    float _vertsPerPoly;

    // Detail mesh generation parameters
    float _detailSampleDist;
    float _detailSampleMaxError;

    // Build options and metrics
    bool _keepInterResults;
    float _totalBuildTimeMs;

    void _cleanup();

    void _setContext(rcContext* ctx) { _ctx = ctx; }
    bool _handleBuild(irr::scene::IMeshSceneNode* node);
    void _resetCommonSettings();
    bool _getMeshBufferData
    (
        irr::scene::IMeshSceneNode* node,
        std::vector<float>& verts, int& nverts,
        std::vector<int>& tris, int& ntris
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
    std::vector<irr::core::vector3df> _returnPath(irr::core::vector3df pStart, irr::core::vector3df pEnd);
};
