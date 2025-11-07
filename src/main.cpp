/*
Game Engine Main Loop
- Group formation movement (Pikmin/Wonderful 101 style)
- Enemies follow player without blocking
- WASD player movement constrained by navmesh
*/
#ifdef _WIN32
#include <windows.h>
#endif
#include <iostream>
#include <irrlicht.h>
#include <memory>
#include <vector>
#include <cmath>
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
    bool KeyIsDown[KEY_KEY_CODES_COUNT]; // Array to hold key states

    PathfindingInputReceiver() : mouseClicked(false) {
        // Initialize all key states to false
        for (u32 i = 0; i < KEY_KEY_CODES_COUNT; ++i)
            KeyIsDown[i] = false;
    }

    virtual bool OnEvent(const SEvent& event) {
        if (event.EventType == EET_MOUSE_INPUT_EVENT) {
            if (event.MouseInput.Event == EMIE_LMOUSE_PRESSED_DOWN) {
                mouseClicked = true;
                mousePos = position2di(event.MouseInput.X, event.MouseInput.Y);
                // std::cout << "Mouse clicked at: " << mousePos.X << ", " << mousePos.Y << std::endl;
                return true;
            }
        }

        // Add key event handling
        if (event.EventType == EET_KEY_INPUT_EVENT) {
            KeyIsDown[event.KeyInput.Key] = event.KeyInput.PressedDown;
            return true;
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

    // Public getter for key states
    virtual bool IsKeyDown(EKEY_CODE keyCode) const {
        return KeyIsDown[keyCode];
    }
};

// Helper function to calculate formation offset
vector3df calculateFormationOffset(int index, int totalCount, float radius) {
    // Create a circular formation around the leader
    const float PI = 3.14159265359f;
    float angle = (2.0f * PI * index) / totalCount;
    return vector3df(
        cosf(angle) * radius,
        0.0f,
        sinf(angle) * radius
    );
}

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

    device->setWindowCaption(L"Game Engine - Group Formation Movement");

    IVideoDriver* driver = device->getVideoDriver();
    ISceneManager* smgr = device->getSceneManager();
    IGUIEnvironment* guienv = device->getGUIEnvironment();

    driver->setTextureCreationFlag(ETCF_CREATE_MIP_MAPS, false);
    driver->setTextureCreationFlag(ETCF_OPTIMIZED_FOR_QUALITY, false);

    smgr->setShadowColor(video::SColor(150, 0, 0, 0));

    /* ===============================
    CAMERA SETUP
    ================================ */
    ICameraSceneNode* camera = smgr->addCameraSceneNode(
        0,
        vector3df(0, 8, 0),
        vector3df(0, 0, 0),
        -1,
        true
    );

    const f32 aspectRatio = (f32)windowWidth / (f32)windowHeight;
    const f32 orthoViewHeight = 20.0f;
    const f32 orthoViewWidth = orthoViewHeight * aspectRatio;

    core::matrix4 orthoMatrix;
    orthoMatrix.buildProjectionMatrixOrthoLH(
        orthoViewWidth,
        orthoViewHeight,
        camera->getNearValue(),
        camera->getFarValue()
    );
    camera->setProjectionMatrix(orthoMatrix, true);

    /* ===============================
    LEVEL MESH SETUP
    ================================ */
    enum
    {
        ID_IsNotPickable = 0,
        IDFlag_IsPickable = 1 << 0,
        IDFlag_IsHighlightable = 1 << 1
    };

    scene::IMesh* levelMesh = smgr->getMesh("assets/test_map_2.obj");
    if (!levelMesh) {
        std::cerr << "Failed to load assets/test_map_2.obj!" << std::endl;
        device->drop();
        return 1;
    }

    scene::ISceneCollisionManager* collMan = nullptr;
    scene::IMeshSceneNode* levelNode = smgr->addMeshSceneNode(levelMesh);
    if (levelNode) {
        levelNode->setMaterialFlag(EMF_LIGHTING, false);
        levelNode->setPosition(core::vector3df(0, 0, 0));
        levelNode->setID(IDFlag_IsPickable);
        levelNode->setVisible(true);

        std::cout << "Creating triangle selector..." << std::endl;
        scene::ITriangleSelector* selector = smgr->createOctreeTriangleSelector(
            levelNode->getMesh(), levelNode, 128
        );

        if (selector) {
            levelNode->setTriangleSelector(selector);
            selector->drop();
            collMan = smgr->getSceneCollisionManager();
            std::cout << "Triangle selector set successfully." << std::endl;
        }
        else {
            std::cerr << "Failed to create triangle selector!" << std::endl;
        }
    }

    /* ===============================
    PATHFINDING SETUP
    ================================ */
    NavMesh* navmesh = new NavMesh(smgr->getRootSceneNode(), smgr, -1);
    NavMeshParams params;

    if (!navmesh->build(levelNode, params)) {
        std::cerr << "Failed to build navigation mesh!" << std::endl;
        device->drop();
        navmesh->drop();
        return 1;
    }

    navmesh->renderNavMesh();

    /* ===============================
    PLAYER SPHERE SETUP
    ================================ */
    IMeshSceneNode* sphere = smgr->addSphereSceneNode(0.2f, 16);
    int sphereAgentId = -1;

    if (sphere) {
        sphere->setPosition(vector3df(0, 1, 0));
        sphere->setMaterialFlag(EMF_LIGHTING, false);
        sphere->setMaterialTexture(0, 0);
        sphere->getMaterial(0).DiffuseColor = SColor(255, 100, 100, 255); // Blue for player

        // Player agent with NO collision avoidance
        dtCrowdAgentParams playerParams;
        memset(&playerParams, 0, sizeof(playerParams));
        playerParams.maxAcceleration = 20.0f;
        playerParams.maxSpeed = 3.5f;
        playerParams.collisionQueryRange = 0.0f; // Disable collision avoidance
        playerParams.updateFlags = DT_CROWD_ANTICIPATE_TURNS | DT_CROWD_OPTIMIZE_VIS | DT_CROWD_OPTIMIZE_TOPO;
        // Note: NO DT_CROWD_OBSTACLE_AVOIDANCE and NO DT_CROWD_SEPARATION

        sphereAgentId = navmesh->addAgent(sphere, playerParams);
        if (sphereAgentId == -1) {
            std::cerr << "Failed to add sphere to crowd!" << std::endl;
        }
    }

    /* ===============================
    ENEMY SETUP (Formation Followers)
    ================================ */
    const int NUM_ENEMIES = 8;
    std::vector<IMeshSceneNode*> enemies;
    std::vector<int> enemyAgentIds;
    const float formationRadius = 1.0f; // Distance from player

    dtCrowdAgentParams followerParams;
    memset(&followerParams, 0, sizeof(followerParams));
    followerParams.maxAcceleration = 15.0f; // Slightly slower acceleration
    followerParams.maxSpeed = 3.0f;         // Slightly slower than player

    // KEY SETTINGS: Enable separation between followers, but not with player
    followerParams.separationWeight = 2.0f; // Push away from other followers
    followerParams.collisionQueryRange = params.AgentRadius * 8.0f;
    followerParams.pathOptimizationRange = params.AgentRadius * 20.0f;

    // Update flags WITHOUT obstacle avoidance for smoother following
    followerParams.updateFlags = DT_CROWD_ANTICIPATE_TURNS |
        DT_CROWD_OPTIMIZE_VIS |
        DT_CROWD_OPTIMIZE_TOPO |
        DT_CROWD_SEPARATION; // Enable separation between followers

    for (int i = 0; i < NUM_ENEMIES; i++) {
        IMeshSceneNode* enemy = smgr->addSphereSceneNode(0.15f, 16);
        if (enemy) {
            // Position enemies in a circle around starting position
            vector3df offset = calculateFormationOffset(i, NUM_ENEMIES, formationRadius);
            enemy->setPosition(vector3df(0, 1, 0) + offset);
            enemy->setMaterialFlag(EMF_LIGHTING, false);

            // Color variation for enemies
            u8 colorVar = (u8)(200 + (i * 5));
            enemy->getMaterial(0).DiffuseColor = SColor(255, colorVar, 0, 0);

            int agentId = navmesh->addAgent(enemy, followerParams);
            if (agentId != -1) {
                enemies.push_back(enemy);
                enemyAgentIds.push_back(agentId);
            }
        }
    }

    /* ===============================
    GUI SETUP
    ================================ */
    guienv->addStaticText(
        L"WASD to move player. Followers form a group around the player.",
        rect<s32>(10, 10, 500, 30),
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
            u32 currentTime = device->getTimer()->getTime();
            f32 deltaTime = (currentTime - lastTime) / 1000.0f;
            lastTime = currentTime;

            // =================================================================
            // NEW: Player WASD Movement Logic
            // =================================================================
            if (sphereAgentId != -1) {
                vector3df moveDir(0, 0, 0);

                // Check key states
                if (inputEventReceiver.IsKeyDown(KEY_KEY_W))
                    moveDir.X += 1.0f; // 'W' moves forward (positive Z)
                if (inputEventReceiver.IsKeyDown(KEY_KEY_S))
                    moveDir.X -= 1.0f; // 'S' moves backward (negative Z)
                if (inputEventReceiver.IsKeyDown(KEY_KEY_A))
                    moveDir.Z += 1.0f; // 'A' moves left (negative X)
                if (inputEventReceiver.IsKeyDown(KEY_KEY_D))
                    moveDir.Z -= 1.0f; // 'D' moves right (positive X)

                vector3df playerPos = sphere->getPosition();

                // Check if any movement key is pressed
                if (moveDir.getLengthSQ() > 0.001f) {
                    moveDir.normalize();

                    // Set a target 1.0 unit away in the direction of movement.
                    // This is updated every frame, simulating continuous velocity.
                    // The agent will move at its maxSpeed towards this point,
                    // constrained by the navmesh.
                    vector3df targetPos = playerPos + moveDir * 1.0f;
                    navmesh->setAgentTarget(sphereAgentId, targetPos);
                }
                else {
                    // No keys pressed: tell the agent to stop by
                    // setting its target to its current position.
                    navmesh->setAgentTarget(sphereAgentId, playerPos);
                }
            }

            // =================================================================
            // OLD: Mouse click handling (REMOVED / COMMENTED OUT)
            // =================================================================
            /*
            if (inputEventReceiver.wasMouseClicked()) {
                position2di mousePos = inputEventReceiver.getMousePos();
                core::line3d<f32> ray = smgr->getSceneCollisionManager()->getRayFromScreenCoordinates(
                    mousePos, camera
                );

                core::vector3df intersectionPoint;
                core::triangle3df hitTriangle;
                scene::ISceneNode* selectedSceneNode =
                    collMan->getSceneNodeAndCollisionPointFromRay(
                        ray, intersectionPoint, hitTriangle, IDFlag_IsPickable, 0
                    );

                if (selectedSceneNode == levelNode && sphereAgentId != -1) {
                    navmesh->setAgentTarget(sphereAgentId, intersectionPoint);
                }
            }
            */

            // Update follower targets to formation positions around the player
            if (sphere && sphereAgentId != -1) {
                vector3df playerPos = sphere->getPosition();

                for (size_t i = 0; i < enemyAgentIds.size(); i++) {
                    // Calculate formation position offset
                    vector3df offset = calculateFormationOffset((int)i, NUM_ENEMIES, formationRadius);
                    vector3df targetPos = playerPos + offset;

                    // Set target with some damping to prevent jitter
                    navmesh->setAgentTarget(enemyAgentIds[i], targetPos);
                }
            }

            navmesh->update(deltaTime);

            // Render scene
            driver->beginScene(true, true, SColor(255, 100, 101, 140));
            smgr->drawAll();
            guienv->drawAll();
            //navmesh->renderAgentPaths(driver);
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