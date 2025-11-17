#include <irrlicht.h>
#include <iostream>
#include <vector>
#include <IrrRecastDetour/StaticNavMesh.h>

#include "../common/InputEventReceiver.h"
#include "../common/Config.h"

// Namespaces
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

#ifdef _IRR_WINDOWS_
#pragma comment(lib, "Irrlicht.lib")
#endif

const u32 windowWidth = Config::WINDOW_WIDTH;
const u32 windowHeight = Config::WINDOW_HEIGHT;

enum
{
    ID_IsNotPickable = 0,
    IDFlag_IsPickable = 1 << 0,
    IDFlag_IsHighlightable = 1 << 1
};

/**
 * @brief Finds the 3D world position of a mouse click on the level geometry.
 * @param smgr Irrlicht Scene Manager.
 * @param camera The active camera.
 * @param mousePos The 2D mouse position from the event receiver.
 * @param mapNode The scene node of the level geometry to test against.
 * @param[out] intersection The 3D point of collision.
 * @return true if the ray hit the mapNode, false otherwise.
 */
bool getMouseWorldPosition(ISceneManager* smgr, ICameraSceneNode* camera, position2di mousePos, ISceneNode* mapNode, vector3df& intersection)
{
    line3df ray = smgr->getSceneCollisionManager()->getRayFromScreenCoordinates(mousePos, camera);
    triangle3df hitTriangle;

    const ISceneNode* hitNode = smgr->getSceneCollisionManager()->getSceneNodeAndCollisionPointFromRay(
        ray,
        intersection,
        hitTriangle,
        IDFlag_IsPickable,
        0);

    // NEW CHECK: See if the hitNode or any of its parents is the mapNode
    const ISceneNode* node = hitNode;
    while (node)
    {
        if (node == mapNode)
        {
            // Success! The ray hit the map or one of its children.
            return true;
        }
        node = node->getParent(); // Move up to the parent
    }

    // The ray hit nothing, or it hit something that isn't part of the mapNode hierarchy
    return false;
}

int main() {
    /*=========================================================
    IRRLICHT SETUP
    =========================================================k*/
    InputEventListener receiver;
    IrrlichtDevice* device = createDevice(
        video::EDT_OPENGL, // Use OpenGL
        dimension2d<u32>(Config::WINDOW_WIDTH, Config::WINDOW_HEIGHT),
        32, false, false, false, &receiver);

    if (!device) {
        std::cerr << "Failed to create Irrlicht device!" << std::endl;
        return 1;
    }
    device->setWindowCaption(L"Irrlicht Recast/Detour Demo - Top Down View");

    IVideoDriver* driver = device->getVideoDriver();
    ISceneManager* smgr = device->getSceneManager();
    IGUIEnvironment* guienv = device->getGUIEnvironment();

    receiver.setGUIEnvironment(guienv);


    /*=========================================================
    LOAD MAP
    =========================================================k*/
    scene::ISceneCollisionManager* levelCollisionManager = nullptr;
    IAnimatedMesh* mapMesh = smgr->getMesh("assets/test_map_2.obj");
    if (!mapMesh) {
        std::cerr << "Failed to load level mesh: assets/test_map_2.obj" << std::endl;
        device->drop();
        return 1;
    }
    IMeshSceneNode* mapNode = smgr->addMeshSceneNode(mapMesh->getMesh(0));

    if (mapNode) {
        mapNode->setMaterialFlag(EMF_LIGHTING, false);
        mapNode->setMaterialFlag(EMF_WIREFRAME, false); // Show map solid
        mapNode->setPosition(vector3df(0, 0, 0));
        mapNode->setID(IDFlag_IsPickable);
        mapNode->setVisible(true);

        scene::ITriangleSelector* selector = smgr->createOctreeTriangleSelector(
            mapNode->getMesh(), mapNode, 128
        );

        if (!selector) {
            std::cerr << "Failed to create triangle selector for level mesh!" << std::endl;
            device->drop();
            return 1;
        }

        if (selector) {
            mapNode->setTriangleSelector(selector);
            selector->drop(); // The node now holds a reference, so we can drop ours
            levelCollisionManager = smgr->getSceneCollisionManager();
            std::cout << "Triangle selector set successfully." << std::endl; // Debug message
        }
        else {
            std::cerr << "Failed to create triangle selector!" << std::endl; // Debug message
        }
    }

    /*=========================================================
    BUILD NAVMESH
    =========================================================k*/
    StaticNavMesh* navMesh = new StaticNavMesh(smgr->getRootSceneNode(), smgr);
    NavMeshParams params; // Use default params from StaticNavMesh.h

    std::cout << "Building navmesh..." << std::endl;
    bool success = navMesh->build(mapNode, params);

    ISceneNode* playerNode = nullptr;
    int playerId = -1;
    std::vector<int> followerIds;
    vector3df lastPlayerPos(0, 0, 0);

    if (success) {
        std::cout << "Navmesh built successfully! Build time: " << navMesh->getTotalBuildTimeMs() << " ms" << std::endl;

        /*=========================================================
        RENDER NAVMESH
        =========================================================k*/
        ISceneNode* debugNavMeshNode = navMesh->renderNavMesh();
        if (debugNavMeshNode) {
            debugNavMeshNode->setMaterialFlag(EMF_LIGHTING, false);
            debugNavMeshNode->setMaterialFlag(EMF_WIREFRAME, true);
            debugNavMeshNode->getMaterial(0).EmissiveColor.set(255, 0, 0, 255); // Blue
        }

        /*=========================================================
        PLAYER AGENT
        =========================================================k*/
        playerNode = smgr->addSphereSceneNode(params.AgentRadius);
        playerNode->setMaterialFlag(EMF_LIGHTING, false);
        playerNode->getMaterial(0).EmissiveColor.set(255, 255, 0, 0); // Red
        vector3df initialPlayerPos(5, 1, 5);
        playerNode->setPosition(initialPlayerPos);
        playerId = navMesh->addAgent(playerNode, params.AgentRadius, params.AgentHeight);

    }
    else {
        std::cerr << "FATAL: Failed to build navmesh!" << std::endl;
    }

    /*=========================================================
    CAMERA - PERSPECTIVE WITH ORBITAL CONTROL
    =========================================================k*/
    // Camera parameters
    float cameraDistance = 15.0f;      // Distance from player
    float cameraAngleH = 0.0f;         // Horizontal angle (yaw) in degrees
    float cameraAngleV = 89.0f;        // Vertical angle (pitch) in degrees - 90 is directly overhead
    const float cameraRotationSpeed = 0.3f; // Sensitivity for mouse drag

    // Create perspective camera
    ICameraSceneNode* camera = smgr->addCameraSceneNode();

    // Set up perspective projection
    const f32 aspectRatio = (f32)windowWidth / (f32)windowHeight;
    camera->setAspectRatio(aspectRatio);
    camera->setFOV(core::degToRad(60.0f));  // 60 degree field of view
    camera->setNearValue(0.1f);
    camera->setFarValue(1000.0f);

    // Set initial camera position (overhead view)
    camera->setPosition(vector3df(0, 15, 0));
    camera->setTarget(vector3df(0, 0, 0));

    /*=========================================================
    GUI
    =========================================================k*/
    IGUIStaticText* stats = guienv->addStaticText(L"", rect<s32>(10, 10, 400, 30));
    stats->setOverrideColor(SColor(255, 255, 255, 255));

    /*=========================================================
    MAIN LOOP
    =========================================================k*/
    u32 then = device->getTimer()->getTime();

    while (device->run()) {
        // Delta time
        u32 now = device->getTimer()->getTime();
        float deltaTime = (float)(now - then) / 1000.0f;
        then = now;

        navMesh->update(deltaTime);

        /*--------------------------------------------------------
        ESC KEY (EXIT)
        --------------------------------------------------------*/
        if (receiver.IsKeyDown(KEY_ESCAPE))
        {
            break;
        }

        /*--------------------------------------------------------
		RIGHT-CLICK CAMERA ROTATION
        --------------------------------------------------------*/
        if (receiver.IsRightMouseDown()) {
            position2di dragDelta = receiver.getMouseDragDelta();

            // Update camera angles based on mouse drag
            cameraAngleH -= dragDelta.X * cameraRotationSpeed;
            cameraAngleV -= dragDelta.Y * cameraRotationSpeed;

            // Clamp vertical angle to prevent flipping
            if (cameraAngleV < 5.0f) cameraAngleV = 5.0f;
            if (cameraAngleV > 89.0f) cameraAngleV = 89.0f;
        }

        /*--------------------------------------------------------
		LEFT-CLICK MOVE PLAYER
        --------------------------------------------------------*/
        if (success && receiver.wasMouseClicked()) {
            position2di mousePos = receiver.getMousePos();

            std::cout << "Mouse clicked at: " << mousePos.X << ", " << mousePos.Y << std::endl;

            core::line3d<f32> ray = smgr->getSceneCollisionManager()->getRayFromScreenCoordinates(
                mousePos,
                camera
            );

            core::vector3df intersectionPoint;
            core::triangle3df hitTriangle;

            scene::ISceneNode* selectedSceneNode =
                levelCollisionManager->getSceneNodeAndCollisionPointFromRay(
                    ray,
                    intersectionPoint, // This will be the position of the collision
                    hitTriangle, // This will be the triangle hit in the collision
                    IDFlag_IsPickable, // This ensures that only nodes that we have
                    // set up to be pickable are considered
                    0); // Check the entire scene

            if (selectedSceneNode) {
                // 5. Check if the node we hit was the levelNode
                //    (This is technically optional if it's your only pickable node)
                if (selectedSceneNode == mapNode) {
                    std::cout << "Mouse clicked mesh at: "
                        << intersectionPoint.X << ", "
                        << intersectionPoint.Y << ", "
                        << intersectionPoint.Z << std::endl;

                    if (playerId != -1) {
                        navMesh->setAgentTarget(playerId, intersectionPoint);
                    }
                }
            }
            else {
                // The ray didn't hit any node *flagged as pickable*
                std::cout << "Mouse ray missed all pickable nodes." << std::endl;
            }
        }

        /*--------------------------------------------------------
        CAMERA FOLLOW PLAYER
        --------------------------------------------------------*/
        if (success && playerNode) {
            vector3df playerPos = playerNode->getPosition();

            // Calculate camera position using spherical coordinates
            float angleHRad = core::degToRad(cameraAngleH);
            float angleVRad = core::degToRad(cameraAngleV);

            // Spherical to Cartesian conversion
            float x = playerPos.X + cameraDistance * sin(angleVRad) * cos(angleHRad);
            float y = playerPos.Y + cameraDistance * cos(angleVRad);
            float z = playerPos.Z + cameraDistance * sin(angleVRad) * sin(angleHRad);

            camera->setPosition(vector3df(x, y, z));
            camera->setTarget(playerPos);
        }

        // --- Render ---
        driver->beginScene(true, true, SColor(255, 20, 20, 30));

        smgr->drawAll();

        // Render agent debug paths
        if (success) {
            navMesh->renderAgentPaths(driver);
        }

        guienv->drawAll();
        driver->endScene();

        // Update stats
        stringw str = L"FPS: "; str += driver->getFPS();
        str += L" | Tris: "; str += driver->getPrimitiveCountDrawn();
        if (success) {
            str += L" | Agents: "; str += (int)followerIds.size() + 1;
        }
        else {
            str += L" | NAVMESH BUILD FAILED";
        }
        stats->setText(str.c_str());
    }

    // 10. Cleanup
    if (navMesh)
        navMesh->drop(); // It's an ISceneNode, so drop it

    device->drop();
    return 0;
}