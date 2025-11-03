/*
Game Engine Main Loop
- Manages multiple pages/screens
- Handles render target texture (RTT) for pixel-perfect rendering
- Uses PageManager to switch between different game states
*/
#ifdef _WIN32
#include <windows.h>
#endif
#include <iostream>
#include <irrlicht.h>
#include <memory>
#include "Config.h"
#include "NavMesh.h"

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

#ifdef _IRR_WINDOWS_
#pragma comment(lib, "Irrlicht.lib")
#endif

// Simple input receiver for pathfinding
class PathfindingInputReceiver : public IEventReceiver {
public:
    bool mouseClicked;
    position2di mousePos;

    PathfindingInputReceiver() : mouseClicked(false) {}

    virtual bool OnEvent(const SEvent& event) {
        if (event.EventType == EET_MOUSE_INPUT_EVENT) {
            if (event.MouseInput.Event == EMIE_LMOUSE_PRESSED_DOWN) {
                mouseClicked = true;
                mousePos = position2di(event.MouseInput.X, event.MouseInput.Y);
                std::cout << "Mouse clicked at: " << mousePos.X << ", " << mousePos.Y << std::endl;
                return true;
            }
        }
        return false;
    }

    bool wasMouseClicked() {
        bool clicked = mouseClicked;
        mouseClicked = false;
        return clicked;
    }

    position2di getMousePos() const {
        return mousePos;
    }
};

int main() {
    /* ===============================
    IRRLICHT SETUP
    ================================ */
    IrrlichtDevice* nulldevice = createDevice(EDT_NULL);
    dimension2d<u32> desktopRes = nulldevice->getVideoModeList()->getDesktopResolution();
    nulldevice->drop();

    const u32 windowWidth = Config::WINDOW_WIDTH;
    const u32 windowHeight = Config::WINDOW_HEIGHT;

    PathfindingInputReceiver inputEventReceiver;

    IrrlichtDevice* device = createDevice(
        EDT_OPENGL,
        dimension2d<u32>(windowWidth, windowHeight),
        32,
        false,
        false,
        false,
        &inputEventReceiver
    );

    if (!device) {
        std::cerr << "Failed to create Irrlicht device!" << std::endl;
        return 1;
    }

    device->setWindowCaption(L"Game Engine - Pathfinding Demo");

    IVideoDriver* driver = device->getVideoDriver();
    ISceneManager* smgr = device->getSceneManager();
    IGUIEnvironment* guienv = device->getGUIEnvironment();

    driver->setTextureCreationFlag(ETCF_CREATE_MIP_MAPS, false);
    driver->setTextureCreationFlag(ETCF_OPTIMIZED_FOR_QUALITY, false);

    smgr->setShadowColor(video::SColor(150, 0, 0, 0)); // Semi-transparent black
    driver->setTextureCreationFlag(ETCF_CREATE_MIP_MAPS, false);

    /* ===============================
    CAMERA SETUP
    ================================ */

    // Orthographic Camera
    ICameraSceneNode* camera = smgr->addCameraSceneNode(
        0,
        vector3df(0, 8, 0),   // Position
        vector3df(0, 0, 0),   // Look-at target
        -1,                   // ID
        true                  // Use orthographic camera
    );

    // Calculate aspect ratio to fix stretching
    const f32 aspectRatio = (f32)windowWidth / (f32)windowHeight;

    // Define the *vertical* size of the view.
    // The horizontal size will be calculated from this.
    const f32 orthoViewHeight = 20.0f; // e.g., our view is 20 units tall
    const f32 orthoViewWidth = orthoViewHeight * aspectRatio; // Calculate width to match aspect ratio

    core::matrix4 orthoMatrix;
    orthoMatrix.buildProjectionMatrixOrthoLH(
        orthoViewWidth,         // Calculated Width
        orthoViewHeight,        // Fixed Height
        camera->getNearValue(), // Near clip plane
        camera->getFarValue()   // Far clip plane
    );
    camera->setProjectionMatrix(orthoMatrix, true); // 'true' for orthographic

    //// Add camera - positioned closer to see the mesh better
    //ICameraSceneNode* camera = smgr->addCameraSceneNode(
    //    0,
    //    vector3df(8, 8, 8),
    //    vector3df(0, 0, 0)
    //);

    //// Set the aspect ratio for the perspective camera
    //camera->setAspectRatio((f32)windowWidth / (f32)windowHeight);

    //scene::ILightSceneNode* light = smgr->addLightSceneNode(
    //    0,
    //    core::vector3df(-10, 30, -15), // Position the light up and to the side
    //    video::SColorf(1.0f, 1.0f, 1.0f), // White light
    //    12.0f // Radius
    //);
    //light->enableCastShadow(true); // Tell the light to cast shadows

    /* ===============================
    LEVEL MESH SETUP (Moved from Pathfinding)
    ================================ */

    enum
    {
        ID_IsNotPickable = 0,
        IDFlag_IsPickable = 1 << 0,
        IDFlag_IsHighlightable = 1 << 1
    };

    // 1. Load the mesh data
    scene::IMesh* levelMesh = smgr->getMesh("assets/test_map_2.obj");
    if (!levelMesh) {
        std::cerr << "Failed to load assets/test_map.obj!" << std::endl;
        device->drop();
        return 1;
    }

    // 2. Create the visible scene node for the level
    scene::ISceneCollisionManager* collMan = nullptr;

    scene::IMeshSceneNode* levelNode = smgr->addMeshSceneNode(levelMesh);
    if (levelNode) {
        levelNode->setMaterialFlag(EMF_LIGHTING, false);
        // Set its position slightly lower, as requested
        levelNode->setPosition(core::vector3df(0, 0, 0));
        levelNode->setID(IDFlag_IsPickable);
        levelNode->setVisible(true);

        std::cout << "Creating triangle selector..." << std::endl; // Debug message
        scene::ITriangleSelector* selector = smgr->createOctreeTriangleSelector(
            levelNode->getMesh(), levelNode, 128
        );

        if (selector) {
            levelNode->setTriangleSelector(selector);
            selector->drop(); // The node now holds a reference, so we can drop ours
            collMan = smgr->getSceneCollisionManager();
            std::cout << "Triangle selector set successfully." << std::endl; // Debug message
        }
        else {
            std::cerr << "Failed to create triangle selector!" << std::endl; // Debug message
        }
    }

    /* ===============================
    PATHFINDING SETUP
    ================================ */

    // Initialize pathfinding system
    NavMesh* navmesh = new NavMesh(smgr->getRootSceneNode(), smgr, -1);

    NavMeshParams params;

    // ...

    if (!navmesh->build(levelNode, params)) {
        std::cerr << "Failed to build navigation mesh!" << std::endl;
        device->drop();
        // Change this:
        // delete navmesh; 
        // To this:
        navmesh->drop(); // <-- USE DROP()
        return 1;
    }

    // 3. (Optional) Now that it's built, create the debug mesh
    //    It no longer needs smgr
    navmesh->createDebugMeshNode();

    /* ===============================
    SPHERE SETUP
    ================================ */

    // Create the sphere that will follow the path
    IMeshSceneNode* sphere = smgr->addSphereSceneNode(0.5f, 16);
    int sphereAgentId = -1;
    int enemy1AgentId = -1;
    int enemy2AgentId = -1;

    if (sphere) {
        sphere->setPosition(vector3df(0, 1, 0));
        sphere->setMaterialFlag(EMF_LIGHTING, true); // 1. Enable lighting
        sphere->setMaterialTexture(0, 0);

        // 2. Adjust material to react to light
        sphere->getMaterial(0).DiffuseColor = SColor(255, 200, 200, 200); // Bright diffuse to catch light
        sphere->getMaterial(0).AmbientColor = SColor(255, 40, 40, 40);   // Dark ambient for shadowed areas
        sphere->getMaterial(0).EmissiveColor = SColor(0, 0, 0, 0);      // No self-illumination
        sphere->getMaterial(0).Shininess = 20.0f;                        // Add a small highlight

        // 3. Tell the sphere to cast shadows
        sphere->addShadowVolumeSceneNode();
        sphereAgentId = navmesh->addAgent(sphere, params.AgentRadius, params.AgentHeight);
        if (sphereAgentId == -1)
        {
            std::cerr << "Failed to add sphere to crowd!" << std::endl;
        }
    }

    /* ===============================
    ENEMY SETUP
    ================================ */

        // Create the first enemy (red cube)
    IMeshSceneNode* enemy1 = smgr->addCubeSceneNode(1.0f); // 1.0f size = 0.5f radius
    if (enemy1) {
        enemy1->setPosition(vector3df(5, 1, 5)); // Place it somewhere on the map
        enemy1->setMaterialFlag(EMF_LIGHTING, false);
        enemy1->getMaterial(0).DiffuseColor = SColor(255, 200, 0, 0); // Red color

        // Add enemy to the crowd
        // Note: We use the same agent params as the player for this demo
        enemy1AgentId = navmesh->addAgent(enemy1, params.AgentRadius, params.AgentHeight);
        if (enemy1AgentId == -1)
        {
            std::cerr << "Failed to add enemy 1 to crowd!" << std::endl;
        }
    }

    // Create the second enemy (red cube)
    IMeshSceneNode* enemy2 = smgr->addCubeSceneNode(1.0f);
    if (enemy2) {
        enemy2->setPosition(vector3df(2, 1, 2)); // Place it somewhere else
        enemy2->setMaterialFlag(EMF_LIGHTING, false);
        enemy2->getMaterial(0).DiffuseColor = SColor(255, 200, 0, 0); // Red color

        // Add enemy to the crowd
        enemy2AgentId = navmesh->addAgent(enemy2, params.AgentRadius, params.AgentHeight);
        if (enemy2AgentId == -1)
        {
            std::cerr << "Failed to add enemy 2 to crowd!" << std::endl;
        }
    }

    /* ===============================
    GUI SETUP
    ================================ */

    // Add instructions text
    guienv->addStaticText(
        L"Left-click on the mesh to move the sphere",
        rect<s32>(10, 10, 400, 30),
        false, true, 0, -1, true
    );

    /* ===============================
    TIMING SETUP
    ================================ */
    u32 lastTime = device->getTimer()->getTime();

    /* ===============================
    MAIN GAME LOOP
    ================================ */
    while (device->run()) {
        if (device->isWindowActive()) {
            // Calculate delta time
            u32 currentTime = device->getTimer()->getTime();
            f32 deltaTime = (currentTime - lastTime) / 1000.0f; // Convert to seconds
            lastTime = currentTime;

            navmesh->update(deltaTime);

			// Mouse click handling for pathfinding
            if (inputEventReceiver.wasMouseClicked()) {
                position2di mousePos = inputEventReceiver.getMousePos();

                // 1. Get the ray from the camera
                core::line3d<f32> ray = smgr->getSceneCollisionManager()->getRayFromScreenCoordinates(
                    mousePos,
                    camera
                );

                // 2. Variables to store the collision result
                core::vector3df intersectionPoint;
                core::triangle3df hitTriangle;

                // 3. Perform collision detection ONLY against pickable nodes
                scene::ISceneNode* selectedSceneNode =
                    collMan->getSceneNodeAndCollisionPointFromRay(
                        ray,
                        intersectionPoint, // This will be the position of the collision
                        hitTriangle, // This will be the triangle hit in the collision
                        IDFlag_IsPickable, // This ensures that only nodes that we have
                        // set up to be pickable are considered
                        0); // Check the entire scene

                // 4. Check if we hit a pickable node
                if (selectedSceneNode) {
                    // 5. Check if the node we hit was the levelNode
                    //    (This is technically optional if it's your only pickable node)
                    if (selectedSceneNode == levelNode) {
                        std::cout << "Mouse clicked mesh at: "
                            << intersectionPoint.X << ", "
                            << intersectionPoint.Y << ", "
                            << intersectionPoint.Z << std::endl;

                        // Move the sphere to the intersection point
                        //sphere->setPosition(intersectionPoint + core::vector3df(0, 0.5f, 0));

                        // Get the sphere's current position *before* moving it
                        //core::vector3df startPos = sphere->getPosition();

                        //// Calculate the path and store it in the member variable 'currentPath'
                        //currentPath = navmesh->findPath(startPos, intersectionPoint);

                        //if (!currentPath.empty()) {
                        //    currentPathIndex = 0;
                        //    isMoving = true;

                        //    std::cout << "Path calculated with " << currentPath.size()
                        //        << " waypoints. Starting movement..." << std::endl;
                        //    std::cout << "Destination: "
                        //        << currentPath.back().X << ", "
                        //        << currentPath.back().Y << ", "
                        //        << currentPath.back().Z << std::endl;
                        //}

                        if (sphereAgentId != -1) {
                            navmesh->setAgentTarget(sphereAgentId, intersectionPoint);
                        }

						navmesh->setAgentTarget(enemy1AgentId, sphere->getPosition());
						navmesh->setAgentTarget(enemy2AgentId, sphere->getPosition());
                    }
                } 
                else {
                    // The ray didn't hit any node *flagged as pickable*
                    std::cout << "Mouse ray missed all pickable nodes." << std::endl;
                }
            }

            // Render scene
            driver->beginScene(true, true, SColor(255, 100, 101, 140));

            // Draw debug path (after drawAll so it renders on top)
            navmesh->renderCrowdDebug(driver);

            smgr->drawAll();
            guienv->drawAll();

            driver->endScene();
        }
        else {
            device->yield();
        }
    }

    /* ===============================
    CLEANUP
    ================================ */
    navmesh->drop();
    device->drop();

    std::cout << "Game exited successfully." << std::endl;
    return 0;
}