/*
Game Engine Main Loop - Wonderful 101 Style Movement
- Direct player control (no navmesh agent for player)
- Swarm follows in trailing formation
- Player constrained to navmesh via manual clamping
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
#include "InputEventReceiver.h"

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

#ifdef _IRR_WINDOWS_
#pragma comment(lib, "Irrlicht.lib")
#endif

// Calculate trailing formation behind leader
vector3df calculateTrailingFormationOffset(int index, int totalCount, float radius, const vector3df& forwardDir) {
    const float PI = 3.14159265359f;

    // Calculate the "back" direction vector
    vector3df backDir = -forwardDir;
    if (backDir.getLengthSQ() < 0.001f) {
        backDir.set(0, 0, -1); // Default to -Z if no forward dir
    }

    // Get the angle of the "back" direction
    float backAngle = atan2f(backDir.Z, backDir.X);

    // Use 270-degree arc (1.5 * PI radians) for spread
    const float totalAngleSpan = 1.5f * PI;

    float followerAngle;
    if (totalCount > 1) {
        float anglePerFollower = totalAngleSpan / (totalCount - 1);
        float startAngle = backAngle - (totalAngleSpan / 2.0f);
        followerAngle = startAngle + (index * anglePerFollower);
    }
    else {
        followerAngle = backAngle;
    }

    return vector3df(
        cosf(followerAngle) * radius,
        0.0f,
        sinf(followerAngle) * radius
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

    InputEventListener inputEventReceiver;

    IrrlichtDevice* device = createDevice(
        EDT_OPENGL,
        dimension2d<u32>(windowWidth, windowHeight),
        32,
        false,
        true,   // vsync on
        false,
        &inputEventReceiver
    );

    if (!device) {
        std::cerr << "Failed to create Irrlicht device!" << std::endl;
        return 1;
    }

    device->setWindowCaption(L"Game Engine - Wonderful 101 Movement");

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
    PLAYER SPHERE SETUP (NO NAVMESH AGENT)
    ================================ */
    IMeshSceneNode* sphere = smgr->addSphereSceneNode(0.2f, 16);

    // Player movement variables
    vector3df playerVelocity(0, 0, 0);
    const float PLAYER_SPEED = 9.5f;
    const float PLAYER_ACCELERATION = 45.0f;
    const float PLAYER_DECELERATION = 35.0f;

    if (sphere) {
        sphere->setPosition(vector3df(0, 1, 0));
        sphere->setMaterialFlag(EMF_LIGHTING, false);
        sphere->setMaterialTexture(0, 0);
        sphere->getMaterial(0).DiffuseColor = SColor(255, 100, 100, 255); // Blue for player
    }

    /* ===============================
    SWARM SETUP (Formation Followers)
    ================================ */
    const int NUM_SWARM = 256;
    std::vector<IMeshSceneNode*> enemies;
    std::vector<int> enemyAgentIds;
    const float formationRadius = 1.5f; // Distance from player

    dtCrowdAgentParams followerParams;
    memset(&followerParams, 0, sizeof(followerParams));
    followerParams.maxAcceleration = 35.0f;
    followerParams.maxSpeed = 10.0f; // Slightly faster than player to catch up

    // Enable separation between followers
    followerParams.separationWeight = 2.5f;
    followerParams.collisionQueryRange = params.AgentRadius * 10.0f;
    followerParams.pathOptimizationRange = params.AgentRadius * 30.0f;

    followerParams.updateFlags = DT_CROWD_ANTICIPATE_TURNS |
        DT_CROWD_OPTIMIZE_VIS |
        DT_CROWD_OPTIMIZE_TOPO |
        DT_CROWD_SEPARATION;

    for (int i = 0; i < NUM_SWARM; i++) {
        IMeshSceneNode* enemy = smgr->addSphereSceneNode(0.15f, 16);
        if (enemy) {
            // Start enemies behind player
            vector3df offset = calculateTrailingFormationOffset(i, NUM_SWARM, formationRadius, vector3df(0, 0, 1));
            enemy->setPosition(vector3df(0, 1, 0) + offset);
            enemy->setMaterialFlag(EMF_LIGHTING, false);

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
        L"WASD to move player. Wonderful 101 style trailing formation.",
        rect<s32>(10, 10, 500, 30),
        false, true, 0, -1, true
    );

    gui::IGUIStaticText* deltaTimeText = guienv->addStaticText(
        L"Delta Time: 0.000",
        rect<s32>(10, 40, 500, 70),
        false, true, 0, -1, true
    );

    gui::IGUIStaticText* fpsText = guienv->addStaticText(
        L"FPS: 00",
        rect<s32>(10, 70, 500, 100),
        false, true, 0, -1, true
    );

    /* ===============================
    TIMING SETUP
    ================================ */
    u32 lastTime = device->getTimer()->getTime();
    vector3df playerForwardDir(0, 0, 1);

    /* ===============================
    MAIN GAME LOOP
    ================================ */
    while (device->run()) {
        if (device->isWindowActive()) {
            u32 currentTime = device->getTimer()->getTime();
            f32 deltaTime = (currentTime - lastTime) / 1000.0f;
            lastTime = currentTime;

            // Cap delta time to prevent large jumps
            if (deltaTime > 0.1f) deltaTime = 0.1f;

            if (deltaTimeText) {
                core::stringw strDelta = L"Delta Time: ";
                strDelta += core::stringw(deltaTime);
                deltaTimeText->setText(strDelta.c_str());
            }

            if (fpsText && driver) {
                core::stringw strFPS = L"FPS: ";
                strFPS += driver->getFPS();
                fpsText->setText(strFPS.c_str());
            }

            // =================================================================
            // DIRECT PLAYER MOVEMENT (NO NAVMESH AGENT)
            // =================================================================
            if (sphere) {
                vector3df inputDir(0, 0, 0);

                // Get input direction
                if (inputEventReceiver.IsKeyDown(KEY_KEY_W))
                    inputDir.X += 1.0f;
                if (inputEventReceiver.IsKeyDown(KEY_KEY_S))
                    inputDir.X -= 1.0f;
                if (inputEventReceiver.IsKeyDown(KEY_KEY_A))
                    inputDir.Z += 1.0f;
                if (inputEventReceiver.IsKeyDown(KEY_KEY_D))
                    inputDir.Z -= 1.0f;

                vector3df playerPos = sphere->getPosition();

                // Apply acceleration/deceleration
                if (inputDir.getLengthSQ() > 0.001f) {
                    inputDir.normalize();
                    playerForwardDir = inputDir;

                    // Accelerate towards input direction
                    vector3df targetVelocity = inputDir * PLAYER_SPEED;
                    playerVelocity += (targetVelocity - playerVelocity) * PLAYER_ACCELERATION * deltaTime;
                }
                else {
                    // Decelerate when no input
                    playerVelocity *= (1.0f - PLAYER_DECELERATION * deltaTime);
                    if (playerVelocity.getLengthSQ() < 0.01f) {
                        playerVelocity.set(0, 0, 0);
                    }
                }

                // Apply velocity
                vector3df newPos = playerPos + playerVelocity * deltaTime;

                // Clamp to navmesh (keeps player on walkable areas)
                vector3df clampedPos = navmesh->getClosestPointOnNavmesh(newPos);

                sphere->setPosition(clampedPos);
            }

            // =================================================================
            // UPDATE SWARM FORMATION
            // =================================================================
            if (sphere) {
                vector3df playerPos = sphere->getPosition();

                for (size_t i = 0; i < enemyAgentIds.size(); i++) {
                    vector3df offset = calculateTrailingFormationOffset(
                        (int)i,
                        NUM_SWARM,
                        formationRadius,
                        playerForwardDir
                    );

                    vector3df targetPos = playerPos + offset;
                    navmesh->setAgentTarget(enemyAgentIds[i], targetPos);
                }
            }

            // Update navmesh agents (swarm only)
            navmesh->update(deltaTime);

            // =================================================================
            // CAMERA FOLLOW
            // =================================================================
            if (sphere && camera) {
                vector3df playerPos = sphere->getPosition();
                const vector3df cameraOffset(0, 8.0f, 0);
                vector3df desiredCameraPos = playerPos + cameraOffset;

                const f32 followSpeed = 8.0f;
                vector3df currentCameraPos = camera->getPosition();
                vector3df newCameraPos = currentCameraPos.getInterpolated(
                    desiredCameraPos,
                    followSpeed * deltaTime
                );

                camera->setPosition(newCameraPos);
                camera->setTarget(playerPos);
            }

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