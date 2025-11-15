

# **Irrlicht Engine NavMesh with Recast & Detour**
## Version 0.2.1

This project integrates the **Recast** (navigation mesh generation) and **Detour** (pathfinding and crowd simulation) libraries into the **Irrlicht Engine**.

It provides a clean, self-contained NavMesh C++ class that wraps the entire Recast/Detour pipeline. This class inherits from irr::scene::ISceneNode, allowing it to be seamlessly added to your Irrlicht scene.

The implementation emphasizes modern C++ by using RAII (std::unique\_ptr with custom deleters) to safely manage the memory of the C-style Recast/Detour libraries.

The included main.cpp demonstrates a complete, interactive demo featuring:

* Runtime navmesh generation from a .obj map.  
* Mouse-click pathfinding for a player agent.  
* Multiple AI agents using DetourCrowd for avoidance and "chase the player" behavior.  
* Debug rendering for the navmesh and agent paths.

## **Dependencies**

* **Irrlicht Engine** (e.g., 1.8.x)  
* **Recast & Detour** libraries (The project files must be configured to link against them)