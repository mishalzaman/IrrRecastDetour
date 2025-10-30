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
#include "ai_Pathfinding.h"
#include "ai_util_RecastDetour.h"

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

    /* ===============================
    SCENE SETUP
    ================================ */

    // Add camera - positioned closer to see the mesh better
    ICameraSceneNode* camera = smgr->addCameraSceneNode(
        0,/// parent node (0 = root/no parent)
        vector3df(0, 12, 0),// position: Directly above the origin (Y=20)
        vector3df(0, 0, 0)// target: Looking down at the origin
    );
    camera->setUpVector(vector3df(0, 0, 1));

    /* ===============================
    LEVEL MESH SETUP (Moved from Pathfinding)
    ================================ */

    // 1. Load the mesh data
    scene::IMesh* levelMesh = smgr->getMesh("media/test_map.obj");
    if (!levelMesh) {
        std::cerr << "Failed to load media/test_map.obj!" << std::endl;
        device->drop();
        return 1;
    }

    // 2. Create the visible scene node for the level
    scene::IMeshSceneNode* levelNode = smgr->addMeshSceneNode(levelMesh);
    if (levelNode) {
        levelNode->setMaterialFlag(EMF_LIGHTING, false);
        // Set its position slightly lower, as requested
        levelNode->setPosition(core::vector3df(0, -0.65f, 0));

        std::cout << "Creating triangle selector..." << std::endl; // Debug message
        scene::ITriangleSelector* selector = smgr->createOctreeTriangleSelector(
            levelNode->getMesh(), levelNode, 128
        );

        if (selector) {
            levelNode->setTriangleSelector(selector);
            selector->drop(); // The node now holds a reference, so we can drop ours
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
    ai_Pathfinding* pathfinding = new ai_Pathfinding();

    std::cout << "Building navigation mesh from levelNode..." << std::endl; // CHANGED

    // CHANGED: Pass the existing 'levelNode' instead of the filename
    if (!pathfinding->load(levelNode, smgr)) {
        std::cerr << "Failed to build navigation mesh!" << std::endl;
        device->drop();
        delete pathfinding;
        return 1;
    }
    std::cout << "Navigation mesh built successfully!" << std::endl; // CHANGED

    /* ===============================
    SPHERE SETUP
    ================================ */

    // Create the sphere that will follow the path
    IMeshSceneNode* sphere = smgr->addSphereSceneNode(0.5f, 16);
    if (sphere) {
        sphere->setPosition(vector3df(0, 1, 0));
        sphere->setMaterialFlag(EMF_LIGHTING, false);
        sphere->setMaterialTexture(0, 0);
        sphere->getMaterial(0).DiffuseColor = SColor(255, 100, 100, 255);
        sphere->getMaterial(0).AmbientColor = SColor(255, 100, 100, 255);
        sphere->getMaterial(0).EmissiveColor = SColor(255, 50, 50, 150);
    }

    std::cout << "Sphere at position: " << sphere->getPosition().X << ", "
              << sphere->getPosition().Y << ", "
		<< sphere->getPosition().Z << std::endl;

    // Variables for movement
    std::vector<vector3df> currentPath;
    int currentPathIndex = 0;
    float moveSpeed = 5.0f; // units per second
    bool isMoving = false;

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

                // 3. Perform collision detection against the *entire scene*
                scene::ISceneNode* hitNode = smgr->getSceneCollisionManager()->getSceneNodeAndCollisionPointFromRay(
                    ray,
                    intersectionPoint, // Output: 3D collision point
                    hitTriangle,       // Output: The triangle that was hit
                    0                  // Default ID mask (check all)
                );

                // 4. Check if we hit anything at all
                if (hitNode) {
                    // 5. Check if the node we hit was the levelNode
                    if (hitNode == levelNode) {
                        std::cout << "Mouse clicked mesh at: "
                            << intersectionPoint.X << ", "
                            << intersectionPoint.Y << ", "
                            << intersectionPoint.Z << std::endl;

                        // Move the sphere to the intersection point
                        sphere->setPosition(intersectionPoint + core::vector3df(0, 0.5f, 0));

                    }
                    else {
                        // We hit a different node (e.g., the sphere)
                        std::cout << "Mouse clicked on a different node." << std::endl;
                    }
                }
                else {
                    // The ray didn't hit any node with a triangle selector
                    std::cout << "Mouse ray missed all nodes." << std::endl;
                }
            }

            // Render scene
            driver->beginScene(true, true, SColor(255, 100, 101, 140));

            smgr->drawAll();
            guienv->drawAll();

            // Draw debug path (after drawAll so it renders on top)
            if (currentPath.size() > 0) {
                pathfinding->renderDebugPath(currentPath, driver);
            }

            driver->endScene();
        }
        else {
            device->yield();
        }
    }

    /* ===============================
    CLEANUP
    ================================ */
    delete pathfinding;
    device->drop();

    std::cout << "Game exited successfully." << std::endl;
    return 0;
}