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
    // We create a temporary NULL device to query the desktop's current resolution.
    IrrlichtDevice* nullDevice = createDevice(video::EDT_NULL);
    dimension2d<u32> deskRes = nullDevice->getVideoModeList()->getDesktopResolution();
    nullDevice->drop();

    // 2. Create Real Device with Desktop Resolution
    // Param 4 set to 'true' enables Fullscreen mode.
    IrrlichtDevice* device = createDevice(
        video::EDT_OPENGL,
        deskRes,
        32,
        false,   // Fullscreen = true
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
        
        // --- TEXTURE FILTERING UPDATE ---
        // Disable all smoothing filters for "Nearest" pixelated look
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
    
    // --- CLIPPING FIX PART 1 ---
    // Increased radius keeps the player center further from walls.
    // 0.4f -> 0.8f pushes the "walkable" edge back, preventing the camera 
    // from getting close enough to clip into the wall geometry.
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
    
    // 1. Standard Camera
    ICameraSceneNode* camera = smgr->addCameraSceneNode();

    // --- FOV UPDATE ---
    camera->setFOV(core::degToRad(60.0f));

    // --- CLIPPING FIX PART 2 ---
    // Set Near Value small so the camera renders geometry even when very close
    camera->setNearValue(0.1f);

    // 2. Player Node (Visual representation)
    ISceneNode* playerNode = smgr->addSphereSceneNode(params.AgentRadius);
    playerNode->setMaterialFlag(EMF_LIGHTING, true);
    playerNode->getMaterial(0).EmissiveColor.set(0, 0, 0, 0); 
    playerNode->getMaterial(0).AmbientColor.set(255, 0, 255, 0); 
    playerNode->getMaterial(0).DiffuseColor.set(255, 0, 255, 0);
    playerNode->setVisible(false); // Hide the green sphere since we are in FPS view
    
    // Initial State
    vector3df playerPos(5, 1, 5);
    playerPos = navMesh->getClosestPointOnNavmesh(playerPos); 
    playerNode->setPosition(playerPos);

    float playerAngle = 0.0f; 
    f32 eyeHeight = 1.2f;

    /*=========================================================
    MAIN LOOP
    =========================================================*/
    u32 then = device->getTimer()->getTime();
    
    // Configuration
    const float MOVEMENT_SPEED = 4.0f;   
    const float TURN_SPEED = 90.0f;     

    while (device->run()) {
        if (receiver.IsKeyDown(KEY_ESCAPE)) break;

        u32 now = device->getTimer()->getTime();
        float deltaTime = (float)(now - then) / 1000.0f;
        then = now;

        if (device->isWindowActive()) {
            
            // --- 1. HANDLE ROTATION (A/D) ---
            if (receiver.IsKeyDown(KEY_KEY_A)) {
                playerAngle -= TURN_SPEED * deltaTime;
            }
            if (receiver.IsKeyDown(KEY_KEY_D)) {
                playerAngle += TURN_SPEED * deltaTime;
            }

            // --- 2. HANDLE MOVEMENT (W/S) ---
            float angleRad = playerAngle * core::DEGTORAD;
            vector3df forwardVec(sin(angleRad), 0, cos(angleRad)); 
            
            vector3df proposedMove(0,0,0);
            bool isMoving = false;

            if (receiver.IsKeyDown(KEY_KEY_W)) {
                proposedMove += forwardVec * MOVEMENT_SPEED * deltaTime;
                isMoving = true;
            }
            if (receiver.IsKeyDown(KEY_KEY_S)) {
                proposedMove -= forwardVec * MOVEMENT_SPEED * deltaTime;
                isMoving = true;
            }

            // --- 3. APPLY PHYSICS & NAVMESH CLAMP ---
            if (isMoving) {
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

            vector3df lookDir(sin(angleRad), 0, cos(angleRad)); 
            camera->setTarget(camPos + lookDir);
        }

        driver->beginScene(true, true, SColor(255, 0, 0, 0)); // Black background
        smgr->drawAll();
        driver->endScene();
    }

    if (navMesh) navMesh->drop();
    device->drop();

    return 0;
}