#include <irrlicht.h>
#include <iostream>
#include <vector>
#include <IrrRecastDetour/CStaticNavMesh.h>

#include <fstream>
#include <sstream>

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

int main() {
    /*=========================================================
    IRRLICHT SETUP
    =========================================================*/
    InputEventListener receiver;

    // 1. Get Desktop Resolution
    IrrlichtDevice* nullDevice = createDevice(video::EDT_NULL);
    dimension2d<u32> deskRes = nullDevice->getVideoModeList()->getDesktopResolution();
    deskRes.Height = (deskRes.Width/4)*3; // Use 4:3 ratio
    nullDevice->drop();

    // 2. Create Real Device
    IrrlichtDevice* device = createDevice(
        video::EDT_OPENGL,
        deskRes,
        32,
        false,   // Fullscreen
        false,
        false,
        &receiver);

    if (!device) {
        std::cerr << "Failed to create Irrlicht device!" << std::endl;
        return 1;
    }
    
    device->setWindowCaption(L"Irrlicht Recast/Detour - Realms Demo");

    IVideoDriver* driver = device->getVideoDriver();
    ISceneManager* smgr = device->getSceneManager();
    smgr->setAmbientLight(SColorf(0.3f, 0.3f, 0.3f, 1.0f));

    /*=========================================================
    NAVMESH & LEVEL SETUP
    =========================================================*/
    IAnimatedMesh* mapMesh = smgr->getMesh("assets/realms/realms.obj");
    if (!mapMesh) {
        std::cerr << "Failed to load level mesh" << std::endl;
        device->drop();
        return 1;
    }
    IMeshSceneNode* mapNode = smgr->addMeshSceneNode(mapMesh->getMesh(0));

    if (mapNode) {
        mapNode->setPosition(vector3df(0, 0, 0));
        mapNode->setMaterialFlag(EMF_NORMALIZE_NORMALS, true);
        
        mapNode->setMaterialFlag(EMF_BILINEAR_FILTER, false);
        mapNode->setMaterialFlag(EMF_TRILINEAR_FILTER, false);
        mapNode->setMaterialFlag(EMF_ANISOTROPIC_FILTER, false);

        for (u32 i = 0; i < mapNode->getMaterialCount(); i++) {
            mapNode->getMaterial(i).Lighting = false;
        }
    }

    // Build NavMesh
    CStaticNavMesh* navMesh = new CStaticNavMesh(smgr->getRootSceneNode(), smgr);
    NavMeshParams params;
    params.CellSize = 0.15f;
    params.CellHeight = 0.2f;
    params.AgentHeight = 1.2f; 
    params.AgentRadius = 0.2f; 
    params.AgentMaxClimb = 0.6f;
    params.AgentMaxSlope = 45.f;
    params.RegionMinSize = 8.f;
    params.RegionMergeSize = 20.f;
    params.EdgeMaxError = 1.3f;
    params.EdgeMaxLen = 12.f; 
    params.VertsPerPoly = 6;
    params.DetailSampleDist = 6.0f;
    params.DetailSampleMaxError = 1.0f;

    bool success = navMesh->build(mapNode, params);

    if (!success) {
        std::cerr << "Navmesh build failed!" << std::endl;
        return 1;
    }

    navMesh->renderNavMesh(); 

    /*=========================================================
    PLAYER & CAMERA SETUP
    =========================================================*/
    ICameraSceneNode* camera = smgr->addCameraSceneNode();
    camera->setFOV(core::degToRad(60.0f));
    camera->setNearValue(0.1f);

    ISceneNode* playerNode = smgr->addSphereSceneNode(params.AgentRadius);
    playerNode->setMaterialFlag(EMF_LIGHTING, true);
    playerNode->getMaterial(0).EmissiveColor.set(0, 0, 0, 0); 
    playerNode->getMaterial(0).AmbientColor.set(255, 0, 255, 0); 
    playerNode->getMaterial(0).DiffuseColor.set(255, 0, 255, 0);
    playerNode->setVisible(false);
    
    // Initial State
    vector3df playerPos(5, 1, 5);
    playerPos = navMesh->getClosestPointOnNavmesh(playerPos); 
    playerNode->setPosition(playerPos);

    float playerAngle = 0.0f; 
    f32 eyeHeight = 1.2f;

    /*=========================================================
    MOVEMENT PHYSICS VARIABLES
    =========================================================*/
    // Settings
    const float MAX_SPEED       = 2.0f;  // Max units per second
    const float ACCELERATION    = 10.0f; // Speed gain per second
    const float DECELERATION    = 15.0f; // Speed loss per second (friction)
    
    const float MAX_TURN_SPEED  = 70.0f;// Max degrees per second
    const float TURN_ACCEL      = 600.0f;// Turn speed gain per second
    const float TURN_DECEL      = 600.0f;// Turn speed loss per second

    // Current Physics State
    float currentSpeed = 0.0f;
    float currentTurnSpeed = 0.0f;

    /*=========================================================
    MAIN LOOP
    =========================================================*/
    u32 then = device->getTimer()->getTime();

    while (device->run()) {
        if (receiver.IsKeyDown(KEY_ESCAPE)) break;

        u32 now = device->getTimer()->getTime();
        float deltaTime = (float)(now - then) / 1000.0f;
        then = now;

        if (device->isWindowActive()) {
            
            // --- 1. HANDLE ROTATION (A/D) ---
            bool isTurning = false;
            
            if (receiver.IsKeyDown(KEY_KEY_A)) {
                currentTurnSpeed -= TURN_ACCEL * deltaTime;
                isTurning = true;
            }
            if (receiver.IsKeyDown(KEY_KEY_D)) {
                currentTurnSpeed += TURN_ACCEL * deltaTime;
                isTurning = true;
            }

            // Apply Turn Deceleration if no key pressed
            if (!isTurning) {
                if (currentTurnSpeed > 0) {
                    currentTurnSpeed -= TURN_DECEL * deltaTime;
                    if (currentTurnSpeed < 0) currentTurnSpeed = 0;
                }
                else if (currentTurnSpeed < 0) {
                    currentTurnSpeed += TURN_DECEL * deltaTime;
                    if (currentTurnSpeed > 0) currentTurnSpeed = 0;
                }
            }

            // Clamp Turn Speed
            if (currentTurnSpeed > MAX_TURN_SPEED) currentTurnSpeed = MAX_TURN_SPEED;
            if (currentTurnSpeed < -MAX_TURN_SPEED) currentTurnSpeed = -MAX_TURN_SPEED;

            // Apply Rotation
            playerAngle += currentTurnSpeed * deltaTime;


            // --- 2. HANDLE MOVEMENT (W/S) ---
            bool isMovingInput = false;

            if (receiver.IsKeyDown(KEY_KEY_W)) {
                currentSpeed += ACCELERATION * deltaTime;
                isMovingInput = true;
            }
            if (receiver.IsKeyDown(KEY_KEY_S)) {
                currentSpeed -= ACCELERATION * deltaTime;
                isMovingInput = true;
            }

            // Apply Movement Deceleration (Friction) if no key pressed
            if (!isMovingInput) {
                if (currentSpeed > 0) {
                    currentSpeed -= DECELERATION * deltaTime;
                    if (currentSpeed < 0) currentSpeed = 0;
                }
                else if (currentSpeed < 0) {
                    currentSpeed += DECELERATION * deltaTime;
                    if (currentSpeed > 0) currentSpeed = 0;
                }
            }

            // Clamp Move Speed
            if (currentSpeed > MAX_SPEED) currentSpeed = MAX_SPEED;
            if (currentSpeed < -MAX_SPEED) currentSpeed = -MAX_SPEED; // Slower backward? Optional.


            // --- 3. APPLY PHYSICS & NAVMESH CLAMP ---
            // Only calculate navmesh collision if we are actually moving
            if (abs(currentSpeed) > 0.001f) {
                float angleRad = playerAngle * core::DEGTORAD;
                vector3df forwardVec(sin(angleRad), 0, cos(angleRad)); 

                vector3df proposedMove = forwardVec * currentSpeed * deltaTime;
                
                vector3df currentPos = playerNode->getPosition();
                vector3df targetPos = currentPos + proposedMove;

                // Check navmesh bounds
                vector3df clampedPos = navMesh->getClosestPointOnNavmesh(targetPos);
                playerNode->setPosition(clampedPos);
            }

            // --- 4. UPDATE CAMERA ---
            vector3df camPos = playerNode->getPosition();
            camPos.Y += eyeHeight;
            camera->setPosition(camPos);

            float angleRad = playerAngle * core::DEGTORAD;
            vector3df lookDir(sin(angleRad), 0, cos(angleRad)); 
            camera->setTarget(camPos + lookDir);
        }

        driver->beginScene(true, true, SColor(255, 0, 0, 0));
        smgr->drawAll();
        driver->endScene();
    }

    if (navMesh) navMesh->drop();
    device->drop();

    return 0;
}