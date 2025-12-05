# IrrRecastDetour
**Version 0.7.28**

A modern C++ integration of the **Recast** (navigation mesh generation) and **Detour** (pathfinding and crowd simulation) libraries into the **Irrlicht Engine**.

## Overview

This project provides clean, self-contained NavMesh classes that wrap the entire Recast/Detour pipeline. These classes inherit from `irr::scene::ISceneNode`, allowing them to be seamlessly integrated into any Irrlicht project as standard scene nodes.

The implementation emphasizes modern C++ practices by using RAII (Resource Acquisition Is Initialization) with `std::unique_ptr` and custom deleters to safely manage the memory of the C-style Recast/Detour libraries.

## Features

- **Two NavMesh Types:**
  - `CStaticNavMesh` - Single mesh build process for small to medium-sized levels
  - `CTiledNavMesh` - Grid-based tiled build process for large worlds with lower memory usage

- **Full Detour Integration:**
  - Advanced pathfinding queries
  - Crowd simulation and steering
  - Agent management with customizable parameters
  - Path distance calculations
  - Target tracking and destination detection

- **Debug Visualization:**
  - Optional wireframe rendering of navigation meshes
  - Agent path visualization
  - Per-tile debug rendering for tiled navmeshes

- **Modern C++ Design:**
  - RAII memory management
  - Smart pointers with custom deleters
  - Clean separation of concerns
  - Intuitive API

## Dependencies

- **Irrlicht Engine** (tested with 1.8.x)
- **Recast & Detour** libraries
- **CMake** 3.10 or higher (for building)
- C++11 compatible compiler

## Setup

1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   cd IrrRecastDetour
   ```

2. **Create build directory:**
   ```bash
   mkdir build
   cd build
   ```

3. **Generate build files:**
   ```bash
   cmake ..
   ```

4. **Build the project:**
   ```bash
   make
   ```

5. **Run examples:**
   ```bash
   ./Demo_Static
   # or
   ./Demo_Tiled
   ```

## Usage Examples

### Basic Static NavMesh

```cpp
#include <IrrRecastDetour/CStaticNavMesh.h>

// Create the navmesh node
CStaticNavMesh* navMesh = new CStaticNavMesh(
    smgr->getRootSceneNode(), 
    smgr
);

// Configure parameters
NavMeshParams params;
params.CellSize = 0.15f;
params.CellHeight = 0.2f;
params.AgentHeight = 0.8f;
params.AgentRadius = 0.4f;
params.AgentMaxClimb = 0.6f;
params.AgentMaxSlope = 45.0f;

// Build navmesh from your level geometry
bool success = navMesh->build(levelMeshNode, params);

// Optional: Create debug visualization
ISceneNode* debugNode = navMesh->renderNavMesh();
```

### Tiled NavMesh for Large Worlds

```cpp
#include <IrrRecastDetour/CTiledNavMesh.h>

CTiledNavMesh* navMesh = new CTiledNavMesh(
    smgr->getRootSceneNode(), 
    smgr
);

NavMeshParams params;
// ... configure params ...
params.KeepInterResults = true; // Required for debug visualization

const int tileSize = 32; // Size in cells
bool success = navMesh->build(levelMeshNode, params, tileSize);
```

### Agent Management

```cpp
// Add an agent
ISceneNode* playerNode = smgr->addSphereSceneNode(0.4f);
int agentId = navMesh->addAgent(playerNode, 0.4f, 0.8f);

// Set movement target
vector3df targetPos(10, 0, 10);
navMesh->setAgentTarget(agentId, targetPos);

// Check if agent reached destination
if (navMesh->HasAgentReachedDestination(agentId)) {
    // Agent has arrived
}

// Get agent velocity
vector3df velocity = navMesh->GetAgentVelocity(agentId);

// Remove agent
navMesh->RemoveAgent(agentId);
```

### Pathfinding Queries

```cpp
// Find path between two points
vector3df start(0, 0, 0);
vector3df end(20, 0, 20);
std::vector<vector3df> path = navMesh->GetPath(start, end);

// Calculate path distance
float distance = navMesh->GetPathDistance(start, end);

// Find closest point on navmesh
vector3df worldPos(5, 2, 5);
vector3df closestPoint = navMesh->getClosestPointOnNavmesh(worldPos);
```

### Rendering Agent Paths

```cpp
// In your main render loop
navMesh->renderAgentPaths(driver);
```

## NavMesh Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `CellSize` | Cell size in world units | 0.3f |
| `CellHeight` | Cell height in world units | 0.2f |
| `AgentHeight` | Agent height | 2.0f |
| `AgentRadius` | Agent radius | 0.6f |
| `AgentMaxClimb` | Maximum climbable height | 0.9f |
| `AgentMaxSlope` | Maximum walkable slope (degrees) | 45.0f |
| `RegionMinSize` | Minimum region size | 8.0f |
| `RegionMergeSize` | Region merge size | 20.0f |
| `EdgeMaxLen` | Maximum edge length | 12.0f |
| `EdgeMaxError` | Maximum edge error | 1.3f |
| `VertsPerPoly` | Vertices per polygon | 6.0f |
| `DetailSampleDist` | Detail sample distance | 6.0f |
| `DetailSampleMaxError` | Detail sample max error | 1.0f |
| `MonotonePartitioning` | Use monotone partitioning | false |
| `KeepInterResults` | Keep intermediate results for debug | false |

## Examples

The project includes two complete example applications:

### Demo_Static
Demonstrates basic static navmesh generation with:
- Single mesh build process
- Agent movement
- Camera controls
- Debug visualization

### Demo_Tiled
Demonstrates tiled navmesh generation with:
- Tile-based mesh building
- Large world support
- Per-tile debug rendering
- Multi-colored tile visualization

Both demos are located in the `examples/` folder and share common input handling code.

## Project Structure

```
IrrRecastDetour/
├── include/IrrRecastDetour/
│   ├── INavMesh.h           # Abstract base class
│   ├── CStaticNavMesh.h     # Static mesh implementation
│   └── CTiledNavMesh.h      # Tiled mesh implementation
├── src/
│   ├── INavMesh.cpp
│   ├── CStaticNavMesh.cpp
│   └── CTiledNavMesh.cpp
├── examples/
│   ├── Demo_Static/
│   ├── Demo_Tiled/
│   └── common/              # Shared code (input, config)
├── assets/                  # Runtime assets
└── CMakeLists.txt
```

## Building Notes

- The project uses CMake for cross-platform building
- Irrlicht library paths must be configured in your CMake environment
- Windows builds automatically copy the Irrlicht DLL to the output directory
- Asset files are copied to the build directory post-build

## License

Please refer to the Recast & Detour library licenses for usage terms of those components.

## Contributing

Contributions are welcome! Please ensure your code follows the existing style and includes appropriate documentation.

## Known Issues

- Debug visualization requires `KeepInterResults = true` for tiled navmeshes
- Agent collision avoidance parameters may need tuning for specific use cases

## Version History

**0.7.28** - Current version
- Modern C++ RAII implementation
- Static and tiled navmesh support
- Full Detour integration
- Debug visualization
- Complete example applications

---

For more information about Recast and Detour, visit the [official repository](https://github.com/recastnavigation/recastnavigation).