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

const bool RENDER_SWARM = true; // Toggle swarm visibility for debugging
const bool RENDER_PLAYER = false; // Toggle player visibility for debugging
const bool RENDER_NAVMESH = false; // Toggle navmesh rendering for debugging

// Random number generator
float randomFloat(float min, float max) {
    return min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
}

// Calculate surrounding ring formation (360 degrees)
vector3df calculateSurroundOffset(int index, int totalCount, float baseRadius) {
    const float PI = 3.14159265359f;

    // 1. Distribute agents evenly around a full circle (0 to 2*PI)
    float angleStep = (2.0f * PI) / (float)totalCount;
    float angle = index * angleStep;

    // 2. Add Randomness (Gish is a blob, not a perfect soldier ring)
    // Vary the radius slightly so they aren't in a perfect hard line
    float randomRadius = baseRadius + randomFloat(-0.3f, 0.3f);

    // Vary the angle slightly for organic overlapping
    float randomAngle = angle + randomFloat(-0.1f, 0.1f);

    // 3. Convert Polar to Cartesian
    return vector3df(
        cosf(randomAngle) * randomRadius,
        0.0f,
        sinf(randomAngle) * randomRadius
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

// ==========================================
// SOFT BODY PHYSICS HELPERS (FIXED)
// ==========================================

struct SoftBodyVertex {
    float currentRadius;
    float velocity;
    float targetRadius;
};

class SoftBodyBlob {
public:
    static const int SEGMENTS = 64;
    SoftBodyVertex vertices[SEGMENTS];
    vector3df visualCenter;
    vector3df centerVelocity;

    // Physics Constants
    float stiffness = 60.0f;
    float damping = 5.0f;       // Increased slightly for stability
    float mass = 1.0f;
    float tension = 120.0f;
    float smoothingFactor = 0.35f;

    SoftBodyBlob() {
        visualCenter.set(0, 0, 0);
        centerVelocity.set(0, 0, 0);
        for (int i = 0; i < SEGMENTS; i++) {
            vertices[i].currentRadius = 1.5f;
            vertices[i].velocity = 0.0f;
            vertices[i].targetRadius = 1.5f;
        }
    }

    // Helper: Intersect ray with segment
    float getRaySegmentIntersection(const vector3df& origin, const vector3df& dir,
        const vector3df& p1, const vector3df& p2) {
        vector3df v1 = origin - p1;
        vector3df v2 = p2 - p1;
        vector3df v3 = vector3df(-dir.Z, 0, dir.X);
        float dot = v2.dotProduct(v3);
        if (abs(dot) < 0.00001f) return -1.0f;
        float t1 = v2.crossProduct(v1).Y / dot;
        float t2 = v1.dotProduct(v3) / dot;
        if (t1 >= 0.0f && (t2 >= 0.0f && t2 <= 1.0f)) return t1;
        return -1.0f;
    }

    void update(float dt, const std::vector<vector3df>& hullPoints, const vector3df& targetCenter, float time) {
        // 1. CENTER PHYSICS (STABILIZED)
        vector3df diff = targetCenter - visualCenter;
        float distSq = diff.getLengthSQ();

        // Spring force
        vector3df centerForce = diff * 80.0f - centerVelocity * 10.0f;
        centerVelocity += centerForce * dt;

        // CAP VELOCITY (Prevents "Explosions" / Jumping)
        // If velocity is too high, the mesh creates a gap between frames
        float speed = centerVelocity.getLength();
        const float MAX_SPEED = 25.0f;
        if (speed > MAX_SPEED) {
            centerVelocity = centerVelocity * (MAX_SPEED / speed);
        }

        visualCenter += centerVelocity * dt;

        // Emergency Teleport only if WAY off (e.g. map change or massive lag spike)
        // Increased from 2.0f to 100.0f (10 units)
        if (distSq > 100.0f) {
            visualCenter = targetCenter;
            centerVelocity.set(0, 0, 0);
        }

        // 2. CALCULATE TARGET SHAPE
        float angleStep = (2.0f * 3.14159f) / SEGMENTS;

        for (int i = 0; i < SEGMENTS; i++) {
            float angle = i * angleStep;
            vector3df dir(cosf(angle), 0, sinf(angle));

            float closestDist = 1000.0f;
            bool hit = false;

            // Optimization: Only check hull points if we are relatively close
            // (prevents checking dist against distant objects if logic breaks)
            for (size_t h = 0; h < hullPoints.size(); h++) {
                vector3df p1 = hullPoints[h];
                vector3df p2 = hullPoints[(h + 1) % hullPoints.size()];
                float dist = getRaySegmentIntersection(visualCenter, dir, p1, p2);
                if (dist > 0.01f && dist < closestDist) {
                    closestDist = dist;
                    hit = true;
                }
            }

            if (hit) {
                vertices[i].targetRadius = closestDist + 0.5f;
            }
            else {
                // If we are outside the hull or ray fails, shrink slowly, don't snap
                if (vertices[i].targetRadius > 0.5f)
                    vertices[i].targetRadius -= 5.0f * dt;
                if (vertices[i].targetRadius < 0.5f)
                    vertices[i].targetRadius = 0.5f;
            }

            // Fluid Ripples
            float noise = sinf(angle * 4.0f + time * 5.0f) * 0.15f;
            vertices[i].targetRadius += noise;
        }

        // 3. INTEGRATE PHYSICS (With Safety Clamps)
        for (int i = 0; i < SEGMENTS; i++) {
            SoftBodyVertex& v = vertices[i];

            float displacement = v.targetRadius - v.currentRadius;
            float force = (stiffness * displacement) - (damping * v.velocity);

            // Neighbor Tension
            int prev = (i - 1 + SEGMENTS) % SEGMENTS;
            int next = (i + 1) % SEGMENTS;
            float neighborAvg = (vertices[prev].currentRadius + vertices[next].currentRadius) / 2.0f;

            // Add tension
            force += (neighborAvg - v.currentRadius) * tension;

            v.velocity += (force / mass) * dt;
            v.currentRadius += v.velocity * dt;

            // Absolute Min/Max clamping to prevent rendering artifacts
            if (v.currentRadius < 0.1f) v.currentRadius = 0.1f;
            if (v.currentRadius > 50.0f) v.currentRadius = 50.0f;
        }

        // 4. SMOOTHING
        for (int pass = 0; pass < 2; pass++) {
            std::vector<float> smoothedRadii(SEGMENTS);
            for (int i = 0; i < SEGMENTS; i++) {
                int prev = (i - 1 + SEGMENTS) % SEGMENTS;
                int next = (i + 1) % SEGMENTS;
                smoothedRadii[i] = vertices[i].currentRadius * (1.0f - smoothingFactor) +
                    (vertices[prev].currentRadius + vertices[next].currentRadius) * (smoothingFactor / 2.0f);
            }
            for (int i = 0; i < SEGMENTS; i++) vertices[i].currentRadius = smoothedRadii[i];
        }
    }
};

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

    if (RENDER_NAVMESH) {
        navmesh->renderNavMesh();
    }

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
        sphere->setVisible(RENDER_PLAYER);
    }

    /* ===============================
    SWARM SETUP (Formation Followers)
    ================================ */
    const int NUM_SWARM = 32;
    std::vector<IMeshSceneNode*> enemies;
    std::vector<int> enemyAgentIds;
    std::vector<vector3df> agentOffsets; // Store random offsets per agent
    const float formationRadius = 1.5f; // Base distance from player

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
        // REMOVED: vector3df(0, 0, 1) argument (direction doesn't matter for a circle)
        vector3df offset = calculateSurroundOffset(i, NUM_SWARM, formationRadius);
        agentOffsets.push_back(offset);
    }

    for (int i = 0; i < NUM_SWARM; i++) {
        IMeshSceneNode* enemy = smgr->addSphereSceneNode(0.15f, 16);
        if (enemy) {
            // Use pre-calculated offset
            enemy->setPosition(vector3df(0, 1, 0) + agentOffsets[i]);
            enemy->setMaterialFlag(EMF_LIGHTING, false);
            enemy->setVisible(RENDER_SWARM);

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
    SoftBodyBlob softBody;

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
            // UPDATE SWARM FORMATION (SURROUND)
            // =================================================================
            if (sphere) {
                vector3df playerPos = sphere->getPosition();

                for (size_t i = 0; i < enemyAgentIds.size(); i++) {
                    // Simple: Target = Player + Offset
                    // We do NOT rotate the offset by playerForwardDir. 
                    // This keeps the "North" agent always North, reducing jitter.

                    vector3df targetPos = playerPos + agentOffsets[i];

                    navmesh->setAgentTarget(enemyAgentIds[i], targetPos);
                }
            }

            // Update navmesh agents (swarm only)
            navmesh->update(deltaTime);

            // =================================================================
            // UPDATE SWARM HULL MESH (LIQUID STYLE)
            // =================================================================
            if (swarmNode && sphere && enemies.size() >= 3) {

                // 1. Calculate Centroid
                vector3df actualCenter(0, 0, 0);
                std::vector<vector3df> swarmPositions;
                swarmPositions.reserve(enemies.size() + 1);

                for (IMeshSceneNode* enemy : enemies) {
                    vector3df pos = enemy->getPosition();
                    swarmPositions.push_back(pos);
                    actualCenter += pos;
                }
                swarmPositions.push_back(sphere->getPosition());
                actualCenter += sphere->getPosition();
                actualCenter /= (float)swarmPositions.size();

                // 2. Calculate Rigid Convex Hull
                std::vector<vector3df> rigidHull = calculateConvexHull(swarmPositions);

                if (rigidHull.size() >= 3) {
                    swarmNode->setVisible(true);

                    // 3. Update Soft Body Physics
                    // Pass total time (in seconds) for ripple effects
                    float timeSec = device->getTimer()->getTime() / 1000.0f;
                    softBody.update(deltaTime, rigidHull, actualCenter, timeSec);

                    // 4. Generate Mesh & CLAMP TO NAVMESH
                    scene::SMeshBuffer* buffer = (scene::SMeshBuffer*)swarmNode->getMesh()->getMeshBuffer(0);
                    buffer->Vertices.clear();
                    buffer->Indices.clear();

                    const float meshY = 0.2f;
                    // Liquid Colors (Darker blue center, lighter rim)
                    const video::SColor centerColor(220, 50, 100, 200);
                    const video::SColor edgeColor(200, 80, 150, 255);

                    // A. Clamp Center (Safety against void)
                    vector3df clampedCenter = navmesh->getClosestPointOnNavmesh(softBody.visualCenter);

                    // If the center dragged too far into a wall, push it out slightly
                    // This prevents the "pancake" effect from being too extreme
                    if (clampedCenter.getDistanceFrom(softBody.visualCenter) > 0.5f) {
                        softBody.visualCenter = clampedCenter;
                        softBody.centerVelocity.set(0, 0, 0); // Kill momentum on impact
                    }

                    buffer->Vertices.push_back(
                        video::S3DVertex(clampedCenter.X, meshY, clampedCenter.Z,
                            0, 1, 0, centerColor, 0.5f, 0.5f)
                    );

                    // B. Add Ring Vertices
                    float angleStep = (2.0f * 3.14159f) / SoftBodyBlob::SEGMENTS;

                    for (int i = 0; i < SoftBodyBlob::SEGMENTS; i++) {
                        float angle = i * angleStep;
                        float r = softBody.vertices[i].currentRadius;

                        vector3df idealPos;
                        idealPos.X = softBody.visualCenter.X + cosf(angle) * r;
                        idealPos.Y = softBody.visualCenter.Y;
                        idealPos.Z = softBody.visualCenter.Z + sinf(angle) * r;

                        vector3df clampedPos = navmesh->getClosestPointOnNavmesh(idealPos);
                        float clampedDist = clampedPos.getDistanceFrom(softBody.visualCenter);

                        // Wall Collision / Squish
                        if (clampedDist < r - 0.05f) { // Tolerance
                            softBody.vertices[i].currentRadius = clampedDist;
                            // Dampen velocity heavily on wall contact (Sticky friction)
                            softBody.vertices[i].velocity *= 0.5f;
                        }

                        // UV Mapping for a circular texture
                        float u = 0.5f + (cosf(angle) * 0.5f);
                        float v = 0.5f + (sinf(angle) * 0.5f);

                        buffer->Vertices.push_back(
                            video::S3DVertex(clampedPos.X, meshY, clampedPos.Z,
                                0, 1, 0, edgeColor, u, v)
                        );
                    }

                    // C. Indices (Triangle Fan)
                    for (u16 i = 0; i < SoftBodyBlob::SEGMENTS; ++i) {
                        buffer->Indices.push_back(0);
                        buffer->Indices.push_back(1 + i);
                        buffer->Indices.push_back(1 + ((i + 1) % SoftBodyBlob::SEGMENTS));
                    }

                    // D. Update Bounding Box (With Padding to prevent flickering)
                    core::aabbox3df bbox;
                    // Initialize with center
                    bbox.reset(softBody.visualCenter);

                    // Add all vertices
                    for (int i = 0; i < SoftBodyBlob::SEGMENTS; i++) {
                        bbox.addInternalPoint(buffer->Vertices[i + 1].Pos);
                    }

                    // CRITICAL FIX: Pad the bounding box!
                    // Physics can move vertices outside the box between frames if fast.
                    // We add a margin of safety so culling doesn't hide the mesh prematurely.
                    vector3df padding(2.0f, 2.0f, 2.0f);
                    bbox.MinEdge -= padding;
                    bbox.MaxEdge += padding;

                    // Ensure Y height covers the mesh
                    bbox.MinEdge.Y = meshY - 1.0f;
                    bbox.MaxEdge.Y = meshY + 1.0f;

                    buffer->BoundingBox = bbox;
                    buffer->setDirty(); // IMPORTANT: Tells Irrlicht to update hardware buffers

                    // Update the SceneNode's box too
                    swarmNode->getMesh()->setBoundingBox(bbox);
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