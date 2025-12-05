

# **Irrlicht Engine NavMesh with Recast & Detour / Version 0.7.27**

This project integrates the **Recast** (navigation mesh generation) and **Detour** (pathfinding and crowd simulation) libraries into the **Irrlicht Engine**.

It provides a clean, self-contained NavMesh C++ class that wraps the entire Recast/Detour pipeline. This class inherits from irr::scene::ISceneNode, allowing it to be seamlessly added to an Irrliccht project.

The implementation emphasizes modern C++ by using RAII (std::unique\_ptr with custom deleters) to safely manage the memory of the C-style Recast/Detour libraries.

## **Dependencies**

* **Irrlicht Engine** (e.g., 1.8.x)  
* **Recast & Detour** libraries (The project files must be configured to link against them)

## **Setup**
* Create build folder
* `cd` in to `build` folder and run `cmake ..`
* Then run `make`
* Examples found in examples folder
