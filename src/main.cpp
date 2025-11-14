#include <irrlicht.h>
#include <iostream>
#include <vector>

#include "StaticNavMesh.h"       // Our navmesh class
#include "InputEventReceiver.h"  // Our input handler
#include "Config.h"              // Window config

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
        0); // ID 0 to check all nodes

    // We only care about clicks on the map geometry
    return (hitNode == mapNode);
}


int main() {
    // 1. Setup Irrlicht Device
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


    // 2. Load Level Geometry
    IAnimatedMesh* mapMesh = smgr->getMesh("assets/test_map_2.obj");
    if (!mapMesh) {
        std::cerr << "Failed to load level mesh: assets/test_map_2.obj" << std::endl;
        device->drop();
        return 1;
    }
    IMeshSceneNode* mapNode = smgr->addMeshSceneNode(mapMesh->getMesh(0));
    mapNode->setMaterialFlag(EMF_LIGHTING, false);
    mapNode->setMaterialFlag(EMF_WIREFRAME, false); // Show map solid
    mapNode->setPosition(vector3df(0, 0, 0));


    // 3. Build StaticNavMesh
    StaticNavMesh* navMesh = new StaticNavMesh(smgr->getRootSceneNode(), smgr);
    NavMeshParams params; // Use default params from StaticNavMesh.h

    std::cout << "Building navmesh..." << std::endl;
    bool success = navMesh->build(mapNode, params);

    ISceneNode* playerNode = nullptr;
    int playerId = -1;
    std::vector<int> followerIds;
    vector3df lastPlayerPos(0, 0, 0);
    ICameraSceneNode* camera = nullptr; // Define camera here for later use

    if (success) {
        std::cout << "Navmesh built successfully! Build time: " << navMesh->getTotalBuildTimeMs() << " ms" << std::endl;

        // 4. Render Navmesh Debug Visual
        ISceneNode* debugNavMeshNode = navMesh->renderNavMesh();
        if (debugNavMeshNode) {
            debugNavMeshNode->setMaterialFlag(EMF_LIGHTING, false);
            debugNavMeshNode->setMaterialFlag(EMF_WIREFRAME, true);
            debugNavMeshNode->getMaterial(0).EmissiveColor.set(255, 0, 0, 255); // Blue
        }

        // 5. Create Player Agent
        playerNode = smgr->addSphereSceneNode(params.AgentRadius);
        playerNode->setMaterialFlag(EMF_LIGHTING, false);
        playerNode->getMaterial(0).EmissiveColor.set(255, 255, 0, 0); // Red
        vector3df initialPlayerPos(10, 5, 10);
        playerNode->setPosition(initialPlayerPos);
        playerId = navMesh->addAgent(playerNode, params.AgentRadius, params.AgentHeight);

        // 6. Create Follower Agents
        const int NUM_FOLLOWERS = 5;
        for (int i = 0; i < NUM_FOLLOWERS; ++i) {
            ISceneNode* followerNode = smgr->addSphereSceneNode(params.AgentRadius * 0.8f);
            followerNode->setMaterialFlag(EMF_LIGHTING, false);
            followerNode->getMaterial(0).EmissiveColor.set(255, 0, 255, 0); // Green

            // Spawn near the player
            vector3df pos = playerNode->getPosition() + vector3df((i - NUM_FOLLOWERS / 2.0f) * 2.0f, 0, 2.0f);
            followerNode->setPosition(pos);

            int followerId = navMesh->addAgent(followerNode, params.AgentRadius * 0.8f, params.AgentHeight);
            if (followerId != -1) {
                followerIds.push_back(followerId);
            }
        }
        std::cout << "Added " << (1 + followerIds.size()) << " agents." << std::endl;
        lastPlayerPos = playerNode->getPosition();

    }
    else {
        std::cerr << "FATAL: Failed to build navmesh!" << std::endl;
        // Continue without AI to allow scene inspection
    }

    // 7. Add Orthographic Camera
    // Get map bounding box to calculate a good ortho view size
    aabbox3d<f32> mapBox = mapNode->getMesh()->getBoundingBox();
    mapNode->getAbsoluteTransformation().transformBox(mapBox);
    vector3df mapCenter = mapBox.getCenter();
    f32 mapWidth = mapBox.getExtent().X;
    f32 mapHeight = mapBox.getExtent().Z;

    // Set view size based on map, with some padding
    f32 orthoViewSize = core::max_(mapWidth, mapHeight) * 1.2f;

    camera = smgr->addCameraSceneNode(
        0, // parent
        vector3df(mapCenter.X, 100.0f, mapCenter.Z), // Initial position high above center
        mapCenter); // Initial target at center

    // Calculate ortho matrix
    matrix4 orthoMatrix;
    f32 orthoHeight = orthoViewSize * (f32)Config::WINDOW_HEIGHT / (f32)Config::WINDOW_WIDTH;
    orthoMatrix.buildProjectionMatrixOrthoLH(orthoViewSize, orthoHeight, 1.0f, 1000.0f);
    camera->setProjectionMatrix(orthoMatrix, true);

    // Set up vector for top-down view (Y is up, so Z is "forward" in 2D)
    camera->setUpVector(vector3df(0, 0, 1));

    // Make cursor visible for clicking
    device->getCursorControl()->setVisible(true);

    // 8. Add GUI for stats
    IGUIStaticText* stats = guienv->addStaticText(L"", rect<s32>(10, 10, 400, 30));
    stats->setOverrideColor(SColor(255, 255, 255, 255));

    // 9. Main Loop
    u32 then = device->getTimer()->getTime();

    while (device->run()) {
        // Delta time
        u32 now = device->getTimer()->getTime();
        float deltaTime = (float)(now - then) / 1000.0f;
        then = now;

        // --- Input ---
        if (receiver.IsKeyDown(KEY_ESCAPE))
            break;

        // Player mouse click
        if (success && receiver.wasMouseClicked()) {
            position2di mousePos = receiver.getMousePos();
            vector3df intersection;

            // Find where the mouse clicked on the level
            if (getMouseWorldPosition(smgr, camera, mousePos, mapNode, intersection)) {
                // Find the closest valid point on the navmesh to the click
                vector3df targetPos = navMesh->getClosestPointOnNavmesh(intersection);

                // Set the player's target
                navMesh->setAgentTarget(playerId, targetPos);
            }
        }

        // --- AI Update ---
        if (success && playerNode) {
            // Update the entire crowd simulation
            navMesh->update(deltaTime);

            // Get player's current position (updated by the crowd)
            vector3df playerPos = playerNode->getPosition();

            // --- Update Camera to Follow Player ---
            camera->setPosition(vector3df(playerPos.X, 100.0f, playerPos.Z)); // 100 units above player
            camera->setTarget(playerPos); // Look at the player

            // Only update follower targets if the player has moved significantly
            if (playerPos.getDistanceFromSQ(lastPlayerPos) > 1.0f) { // 1 unit threshold
                for (int followerId : followerIds) {
                    navMesh->setAgentTarget(followerId, playerPos);
                }
                lastPlayerPos = playerPos;
            }
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