/*
Game Engine Main Loop - Wonderful 101 Style Movement
- Direct player control (no navmesh agent for player)
- Swarm follows in trailing formation
- Player constrained to navmesh via manual clamping
- DYNAMIC MESH attached to swarm hull
*/
#ifdef _WIN32
#include <windows.h>
#endif
#include <iostream>
#include <irrlicht.h>
#include <memory>
#include <vector>
#include <cmath>
#include <ctime>
#include <algorithm> // For std::sort (Convex Hull)
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

// Random number generator
float randomFloat(float min, float max) {
    return min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
}

// Calculate random cluster formation behind leader
vector3df calculateClusterFormationOffset(int index, int totalCount, float baseRadius, const vector3df& forwardDir) {
    const float PI = 3.14159265359f;

    // Calculate the "back" direction vector
    vector3df backDir = -forwardDir;
    if (backDir.getLengthSQ() < 0.001f) {
        backDir.set(0, 0, -1); // Default to -Z if no forward dir
    }

    // Get the angle of the "back" direction
    float backAngle = atan2f(backDir.Z, backDir.X);

    // Create clusters with random variation
    // Use concentric rings with random angular distribution
    int ring = (index / 16); // 16 agents per ring

    // Random radius within ring bounds
    float minRadius = baseRadius + (ring * 0.8f);
    float maxRadius = baseRadius + (ring * 0.8f) + 0.6f;
    float radius = randomFloat(minRadius, maxRadius);

    // Random angle with bias toward back
    // Create a 180-degree arc behind the player with random distribution
    float angleRange = PI; // 180 degrees
    float randomAngle = randomFloat(-angleRange / 2.0f, angleRange / 2.0f);
    float followerAngle = backAngle + randomAngle;

    // Add some random offset for natural clustering
    float offsetX = randomFloat(-0.2f, 0.2f);
    float offsetZ = randomFloat(-0.2f, 0.2f);

    return vector3df(
        cosf(followerAngle) * radius + offsetX,
        0.0f,
        sinf(followerAngle) * radius + offsetZ
    );
}

// 2D Cross product (on XZ plane)
// > 0 for counter-clockwise turn
// < 0 for clockwise turn
// = 0 for collinear
float cross_product_xz(const vector3df& o, const vector3df& a, const vector3df& b) {
    return (a.X - o.X) * (b.Z - o.Z) - (a.Z - o.Z) * (b.X - o.X);
}

// Calculate 2D convex hull (Monotone Chain algorithm)
std::vector<vector3df> calculateConvexHull(std::vector<vector3df>& points) {
    int n = points.size();
    if (n < 3) return {}; // Need at least 3 points for a hull

    // Sort points lexicographically (by X, then Z)
    std::sort(points.begin(), points.end(), [](const vector3df& a, const vector3df& b) {
        return a.X < b.X || (a.X == b.X && a.Z < b.Z);
        });

    std::vector<vector3df> hull;

    // Build lower hull
    for (int i = 0; i < n; ++i) {
        // Pop points that make a clockwise turn
        while (hull.size() >= 2 && cross_product_xz(hull[hull.size() - 2], hull.back(), points[i]) <= 0) {
            hull.pop_back();
        }
        hull.push_back(points[i]);
    }

    // Build upper hull
    int lowerHullSize = hull.size();
    for (int i = n - 2; i >= 0; --i) {
        // Pop points that make a clockwise turn
        while (hull.size() > lowerHullSize && cross_product_xz(hull[hull.size() - 2], hull.back(), points[i]) <= 0) {
            hull.pop_back();
        }
        hull.push_back(points[i]);
    }

    // Remove last point (it's the same as the first)
    hull.pop_back();

    return hull;
}


int main() {
    // Seed random number generator for formation variation
    srand(static_cast<unsigned int>(time(nullptr)));

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
    const int NUM_SWARM = 16;
    std::vector<IMeshSceneNode*> enemies;
    std::vector<int> enemyAgentIds;
    std::vector<vector3df> agentOffsets; // Store random offsets per agent
    const float formationRadius = 1.0f; // Base distance from player

    dtCrowdAgentParams followerParams;
    memset(&followerParams, 0, sizeof(followerParams));
    followerParams.maxAcceleration = 60.0f;
    followerParams.maxSpeed = 12.0f; // Slightly faster than player to catch up

    // Enable separation between followers
    followerParams.separationWeight = 2.5f;
    followerParams.collisionQueryRange = params.AgentRadius * 10.0f;
    followerParams.pathOptimizationRange = params.AgentRadius * 30.0f;

    followerParams.updateFlags = DT_CROWD_ANTICIPATE_TURNS |
        DT_CROWD_OPTIMIZE_VIS |
        DT_CROWD_OPTIMIZE_TOPO |
        DT_CROWD_SEPARATION;

    // Pre-calculate random offsets for each agent
    for (int i = 0; i < NUM_SWARM; i++) {
        vector3df offset = calculateClusterFormationOffset(i, NUM_SWARM, formationRadius, vector3df(0, 0, 1));
        agentOffsets.push_back(offset);
    }

    for (int i = 0; i < NUM_SWARM; i++) {
        IMeshSceneNode* enemy = smgr->addSphereSceneNode(0.15f, 16);
        if (enemy) {
            // Use pre-calculated offset
            enemy->setPosition(vector3df(0, 1, 0) + agentOffsets[i]);
            enemy->setMaterialFlag(EMF_LIGHTING, false);
            //enemy->setVisible(false);

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
     SWARM HULL MESH SETUP
    ================================ */
    scene::SMesh* swarmMesh = new scene::SMesh();
    // Use CDynamicMeshBuffer for efficient per-frame updates
    // Note: In 1.8.5, CDynamicMeshBuffer is just a typedef for SMeshBuffer
    // but we still need to get the buffer from the mesh.
    scene::IMeshBuffer* swarmBuffer = new scene::SMeshBuffer();
    swarmMesh->addMeshBuffer(swarmBuffer);
    swarmBuffer->drop(); // Mesh now owns the buffer

    scene::IMeshSceneNode* swarmNode = smgr->addMeshSceneNode(swarmMesh);
    swarmMesh->drop(); // Node now owns the mesh

    if (swarmNode) {
        swarmNode->setMaterialFlag(EMF_LIGHTING, false);
        swarmNode->setMaterialFlag(EMF_BACK_FACE_CULLING, false); // See both sides

        // Use EMF_WIREFRAME for debugging the hull shape
        // swarmNode->setMaterialFlag(EMF_WIREFRAME, true); 

        // Set to a semi-transparent material
        swarmNode->getMaterial(0).DiffuseColor = SColor(150, 150, 200, 255); // Semi-transparent blue/white
        swarmNode->setMaterialType(EMT_TRANSPARENT_ALPHA_CHANNEL);
        swarmNode->setVisible(false); // Hide until we have a valid hull
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

                // Recalculate formation offsets based on current movement direction
                for (size_t i = 0; i < enemyAgentIds.size(); i++) {
                    // Use stored offset but rotate it based on player's direction
                    vector3df baseOffset = agentOffsets[i];

                    // Get angle of player's forward direction
                    float playerAngle = atan2f(playerForwardDir.Z, playerForwardDir.X);
                    float baseAngle = atan2f(baseOffset.Z, baseOffset.X);
                    float baseRadius = sqrtf(baseOffset.X * baseOffset.X + baseOffset.Z * baseOffset.Z);

                    // Rotate offset to match player direction
                    float newAngle = baseAngle + playerAngle - (3.14159265359f / 2.0f); // Adjust for initial orientation
                    vector3df rotatedOffset(
                        cosf(newAngle) * baseRadius,
                        0.0f,
                        sinf(newAngle) * baseRadius
                    );

                    vector3df targetPos = playerPos + rotatedOffset;
                    navmesh->setAgentTarget(enemyAgentIds[i], targetPos);
                }
            }

            // Update navmesh agents (swarm only)
            navmesh->update(deltaTime);

            // =================================================================
            // UPDATE SWARM HULL MESH (with NavMesh Boundary Clipping)
            // =================================================================
            if (swarmNode && sphere && enemies.size() >= 3) {
                // 1. Collect all swarm agent positions (they're already clamped to navmesh)
                std::vector<vector3df> swarmPositions;
                swarmPositions.reserve(enemies.size() + 1);

                for (IMeshSceneNode* enemy : enemies) {
                    swarmPositions.push_back(enemy->getPosition());
                }

                // Add player position
                swarmPositions.push_back(sphere->getPosition());

                // 2. Calculate the 2D convex hull from swarm positions
                std::vector<vector3df> hull = calculateConvexHull(swarmPositions);

                if (hull.size() < 3) {
                    swarmNode->setVisible(false);
                }
                else {
                    // 3. Clamp all hull vertices to navmesh and validate
                    std::vector<vector3df> clampedHull;
                    clampedHull.reserve(hull.size() * 2); // Reserve extra space for edge insertions

                    for (size_t i = 0; i < hull.size(); ++i) {
                        vector3df currentPoint = hull[i];
                        vector3df nextPoint = hull[(i + 1) % hull.size()];

                        // Clamp current point to navmesh
                        vector3df clampedCurrent = navmesh->getClosestPointOnNavmesh(currentPoint);

                        // Check if current point moved significantly (it was off navmesh)
                        float currentDistSq = (clampedCurrent - currentPoint).getLengthSQ();
                        bool currentWasOffMesh = currentDistSq > 0.01f;

                        // Add the clamped current point
                        clampedHull.push_back(clampedCurrent);

                        // Check edge between current and next point
                        // Sample points along the edge to detect if it crosses navmesh boundary
                        const int edgeSamples = 8;
                        vector3df lastValidPoint = clampedCurrent;

                        for (int j = 1; j < edgeSamples; ++j) {
                            float t = (float)j / (float)edgeSamples;
                            vector3df edgePoint = currentPoint.getInterpolated(nextPoint, t);
                            vector3df clampedEdge = navmesh->getClosestPointOnNavmesh(edgePoint);

                            float edgeDistSq = (clampedEdge - edgePoint).getLengthSQ();

                            // If this edge point is off navmesh, insert the boundary point
                            if (edgeDistSq > 0.01f) {
                                // Only add if it's different enough from last point
                                float diffSq = (clampedEdge - lastValidPoint).getLengthSQ();
                                if (diffSq > 0.04f) { // Minimum spacing between points
                                    clampedHull.push_back(clampedEdge);
                                    lastValidPoint = clampedEdge;
                                }
                            }
                        }
                    }

                    // 4. Remove duplicate points (points that are too close together)
                    std::vector<vector3df> finalHull;
                    finalHull.reserve(clampedHull.size());

                    for (size_t i = 0; i < clampedHull.size(); ++i) {
                        bool isDuplicate = false;
                        for (size_t j = 0; j < finalHull.size(); ++j) {
                            float distSq = (clampedHull[i] - finalHull[j]).getLengthSQ();
                            if (distSq < 0.01f) { // Too close, consider duplicate
                                isDuplicate = true;
                                break;
                            }
                        }
                        if (!isDuplicate) {
                            finalHull.push_back(clampedHull[i]);
                        }
                    }

                    // 5. Recalculate convex hull from clamped points (they may no longer be convex)
                    if (finalHull.size() >= 3) {
                        hull = calculateConvexHull(finalHull);
                    }
                    else {
                        hull = finalHull;
                    }

                    if (hull.size() < 3) {
                        swarmNode->setVisible(false);
                    }
                    else {
                        swarmNode->setVisible(true);

                        // 6. Get the mesh buffer
                        scene::SMeshBuffer* buffer = (scene::SMeshBuffer*)swarmNode->getMesh()->getMeshBuffer(0);

                        // 7. Clear buffers
                        buffer->Vertices.clear();
                        buffer->Indices.clear();

                        // 8. Calculate swarm center (use actual agent positions, not hull)
                        vector3df swarmCenter(0, 0, 0);
                        for (IMeshSceneNode* enemy : enemies) {
                            swarmCenter += enemy->getPosition();
                        }
                        swarmCenter += sphere->getPosition();
                        swarmCenter /= (float)(enemies.size() + 1);

                        // 9. Triangulate the hull (fan from swarm center)
                        const video::SColor vColor(255, 150, 200, 255);
                        const float meshY = 0.1f;

                        // Add center vertex
                        buffer->Vertices.push_back(
                            video::S3DVertex(swarmCenter.X, meshY, swarmCenter.Z,
                                0, 1, 0, vColor, 0.5f, 0.5f)
                        );
                        u16 centerIndex = 0;

                        // Add all hull vertices
                        for (const auto& v : hull) {
                            buffer->Vertices.push_back(
                                video::S3DVertex(v.X, meshY, v.Z,
                                    0, 1, 0, vColor, 0.0f, 0.0f)
                            );
                        }

                        // 10. Create indices for the triangle fan
                        u16 hullVertexCount = (u16)hull.size();
                        for (u16 i = 0; i < hullVertexCount; ++i) {
                            u16 v1_idx = 1 + i;
                            u16 v2_idx = 1 + ((i + 1) % hullVertexCount);

                            buffer->Indices.push_back(centerIndex);
                            buffer->Indices.push_back(v1_idx);
                            buffer->Indices.push_back(v2_idx);
                        }

                        // 11. Set bounding box
                        core::aabbox3df bbox;
                        bbox.reset(swarmCenter);
                        for (const auto& v : hull) {
                            bbox.addInternalPoint(v);
                        }
                        bbox.MinEdge.Y = meshY - 0.5f;
                        bbox.MaxEdge.Y = meshY + 0.5f;
                        buffer->BoundingBox = bbox;

                        // 12. Update mesh
                        buffer->setDirty();
                        swarmNode->getMesh()->setBoundingBox(bbox);
                    }
                }
            }

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