

# **Irrlicht Engine NavMesh with Recast & Detour**

This project integrates the **Recast** (navigation mesh generation) and **Detour** (pathfinding and crowd simulation) libraries into the **Irrlicht Engine**.

It provides a clean, self-contained NavMesh C++ class that wraps the entire Recast/Detour pipeline. This class inherits from irr::scene::ISceneNode, allowing it to be seamlessly added to your Irrlicht scene.

The implementation emphasizes modern C++ by using RAII (std::unique\_ptr with custom deleters) to safely manage the memory of the C-style Recast/Detour libraries.

The included main.cpp demonstrates a complete, interactive demo featuring:

* Runtime navmesh generation from a .obj map.  
* Mouse-click pathfinding for a player agent.  
* Multiple AI agents using DetourCrowd for avoidance and "chase the player" behavior.  
* Debug rendering for the navmesh and agent paths.

## **Features**

* **Scene Node Integration**: The NavMesh class is a proper ISceneNode (though invisible), so it exists naturally within the Irrlicht scene graph and is managed by the ISceneManager.  
* **Automatic Geometry Extraction**: Builds a navmesh from any standard IMeshSceneNode. It automatically extracts all vertex and index data from its mesh buffers and applies the node's absolute transformation.  
* **Full Recast Pipeline**: Implements the complete Recast generation pipeline (steps 1-9), from voxelization to detailed mesh generation.  
* **Detour Pathfinding**:  
  * Provides a simple A\* path query (findPath).  
  * Implements the **DetourCrowd** system for managing multiple agents.  
* **Crowd Simulation**:  
  * Agents are linked directly to ISceneNode pointers.  
  * Handles pathfinding, agent-to-agent avoidance, and following.  
  * The NavMesh::update(deltaTime) function ticks the simulation and automatically updates the positions of all linked Irrlicht nodes.  
* **Debug Visualization**:  
  * createDebugMeshNode(): Generates a new IMeshSceneNode (wireframe) to visualize the generated navmesh.  
  * renderCrowdDebug(): Draws 3D lines showing the current path (corridors) for all agents in the crowd.  
* **Safe Memory Management**: Uses std::unique\_ptr and custom deleters for all Recast/Detour objects, eliminating manual rcFree\* or dtFree\* calls and preventing memory leaks.

## **Core Components**

* **NavMesh.h / NavMesh.cpp**: The core library. This single class encapsulates all Recast/Detour logic. It holds the build configuration, the build data, the final dtNavMesh, the dtNavMeshQuery, and the dtCrowd simulation.  
* **main.cpp**: A comprehensive demo showing how to initialize, build, and use the NavMesh node with multiple agents.  
* **NavMeshParams (struct)**: A simple struct defined in NavMesh.h that holds all configuration parameters for the navmesh build (cell size, agent height, slope, etc.).

## **How It Works: The Pipeline**

1. **Initialization**:  
   * An IMeshSceneNode (e.g., from a .obj or .bsp file) is loaded into Irrlicht. This is the "level."  
   * A NavMesh node is created: new NavMesh(smgr-\>getRootSceneNode(), smgr).  
2. **Building**:  
   * navmesh-\>build(levelNode, params) is called.  
   * **Irrlicht \-\> Recast**: The private \_getMeshBufferData function iterates the levelNode's mesh buffers, extracts all vertices and indices, and copies them into std::vectors of floats and ints.  
   * **Recast Build**: The standard Recast pipeline is executed (voxelize, filter, build regions, contours, polys, and detail mesh).  
   * **Recast \-\> Detour**: The resulting polygon and detail mesh data is used to create the final dtNavMesh.  
   * **Init Simulation**: A dtNavMeshQuery and dtCrowd object are initialized with the new dtNavMesh.  
3. **Adding Agents**:  
   * An ISceneNode (like a sphere or animated mesh) is created in Irrlicht to represent an agent.  
   * navmesh-\>addAgent(agentNode, radius, height) is called.  
   * This adds a new dtCrowdAgent to the simulation and stores a link between the agent's ID and the ISceneNode\* in an internal std::map.  
4. **Setting Targets**:  
   * navmesh-\>setAgentTarget(agentId, targetPos) is called (e.g., from a mouse click or AI logic).  
   * The NavMesh class finds the nearest dtPolyRef to the targetPos and tells the dtCrowd agent to move there. dtCrowd automatically calculates the path.  
5. **Per-Frame Update (The Magic)**:  
   * Inside the main game loop, navmesh-\>update(deltaTime) is called once per frame.  
   * This single call does two things:  
     1. It calls \_crowd-\>update(deltaTime, ...), which runs the Detour simulation. dtCrowd moves all agents along their paths and handles local avoidance.  
     2. It iterates its internal \_agentNodeMap, gets the new position (npos) of each active agent, and applies it to the corresponding Irrlicht ISceneNode using node-\>setPosition().

## **Dependencies**

* **Irrlicht Engine** (e.g., 1.8.x)  
* **Recast & Detour** libraries (The project files must be configured to link against them)