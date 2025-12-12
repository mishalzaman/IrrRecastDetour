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

// Framebuffer Dimensions (4:3 Ratio)
const u32 FB_WIDTH = 640;
const u32 FB_HEIGHT = 480;

// ---------------------------------------------------------------------------
// 1. BASE SHADERS (GLSL)
// ---------------------------------------------------------------------------
const c8* VERTEX_SHADER = R"(
void main()
{
    gl_Position = ftransform();
    gl_TexCoord[0] = gl_MultiTexCoord0;
}
)";

const c8* FRAGMENT_SHADER = R"(
uniform sampler2D RTT;

void main()
{
    vec2 uv = gl_TexCoord[0].xy;
    vec4 color = texture2D(RTT, uv);
    gl_FragColor = color;
}
)";

// ---------------------------------------------------------------------------
// 2. SHADER CALLBACK
// ---------------------------------------------------------------------------
class ShaderCallBack : public video::IShaderConstantSetCallBack
{
public:
    virtual void OnSetConstants(video::IMaterialRendererServices* services, s32 userData)
    {
        // Set the Texture Sampler to index 0 (First texture layer)
        s32 TextureLayerID = 0;
        services->setPixelShaderConstant("RTT", &TextureLayerID, 1);
    }
};


int main() {
    /*=========================================================
    IRRLICHT SETUP
    =========================================================*/
    InputEventListener receiver;

    // 1. Get Desktop Resolution
    IrrlichtDevice* nullDevice = createDevice(video::EDT_NULL);
    dimension2d<u32> deskRes = nullDevice->getVideoModeList()->getDesktopResolution();
    deskRes.Height = (deskRes.Width/4)*3; // Force 4:3 Ratio
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

    // Disable all mip-map textures
    driver->setTextureCreationFlag(irr::video::ETCF_CREATE_MIP_MAPS, false);

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
            mapNode->getMaterial(i).FogEnable = true;
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
    vector3df playerRotation(1, -90, 0);
    playerPos = navMesh->getClosestPointOnNavmesh(playerPos); 
    playerNode->setPosition(playerPos);
    playerNode->setRotation(playerRotation);

    float playerAngle = -90.0f; 
    f32 eyeHeight = 1.2f;

    /*=========================================================
    FRAMEBUFFER & SHADER SETUP
    =========================================================*/
    
    // Framebuffer Dimensions (4:3 Ratio)
    const u32 FB_WIDTH = deskRes.Width;
    const u32 FB_HEIGHT = deskRes.Height;

    // 1. Create Render Target Texture (320x240)
    ITexture* rtt = driver->addRenderTargetTexture(
        dimension2d<u32>(FB_WIDTH, FB_HEIGHT), 
        "RTT_Base"
    );

    // 2. Compile Shader Material
    video::IGPUProgrammingServices* gpu = driver->getGPUProgrammingServices();
    s32 shaderMaterialType = 0;

    if (gpu) {
        ShaderCallBack* mc = new ShaderCallBack();
        
        shaderMaterialType = gpu->addHighLevelShaderMaterial(
            VERTEX_SHADER, "main", video::EVST_VS_1_1,
            FRAGMENT_SHADER, "main", video::EPST_PS_1_1,
            mc, video::EMT_SOLID, 0);

        mc->drop();
    }

    // 3. Create Screen Quad Geometry
    // V coordinates are flipped (0.0 at top) to fix upside-down RTT issue
    S3DVertex quadVertices[4];
    quadVertices[0] = S3DVertex(-1.0f, -1.0f, 0.0f, 0,0,1, SColor(255,255,255,255), 0.0f, 0.0f); 
    quadVertices[1] = S3DVertex(-1.0f,  1.0f, 0.0f, 0,0,1, SColor(255,255,255,255), 0.0f, 1.0f); 
    quadVertices[2] = S3DVertex( 1.0f,  1.0f, 0.0f, 0,0,1, SColor(255,255,255,255), 1.0f, 1.0f); 
    quadVertices[3] = S3DVertex( 1.0f, -1.0f, 0.0f, 0,0,1, SColor(255,255,255,255), 1.0f, 0.0f); 
    
    u16 quadIndices[] = { 0, 1, 2, 0, 2, 3 };

    // 4. Set Material Properties
    SMaterial quadMaterial;
    quadMaterial.MaterialType = (video::E_MATERIAL_TYPE)shaderMaterialType;
    quadMaterial.TextureLayer[0].Texture = rtt;
    // Disable filtering to keep sharp pixels when scaling up
    quadMaterial.TextureLayer[0].BilinearFilter = false; 
    quadMaterial.TextureLayer[0].TrilinearFilter = false;
    quadMaterial.TextureLayer[0].AnisotropicFilter = 0;
    quadMaterial.TextureLayer[0].TextureWrapU = video::ETC_CLAMP_TO_EDGE;
    quadMaterial.TextureLayer[0].TextureWrapV = video::ETC_CLAMP_TO_EDGE;
    quadMaterial.Lighting = false;

    /*=========================================================
    MOVEMENT PHYSICS VARIABLES
    =========================================================*/
    const float MAX_SPEED       = 2.0f;  
    const float ACCELERATION    = 10.0f; 
    const float DECELERATION    = 15.0f; 
    
    const float MAX_TURN_SPEED  = 90.0f;
    const float TURN_ACCEL      = 600.0f;
    const float TURN_DECEL      = 600.0f;

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
            
            // --- PHYSICS LOGIC (Unchanged) ---
            bool isTurning = false;
            if (receiver.IsKeyDown(KEY_KEY_A)) {
                currentTurnSpeed -= TURN_ACCEL * deltaTime;
                isTurning = true;
            }
            if (receiver.IsKeyDown(KEY_KEY_D)) {
                currentTurnSpeed += TURN_ACCEL * deltaTime;
                isTurning = true;
            }
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
            if (currentTurnSpeed > MAX_TURN_SPEED) currentTurnSpeed = MAX_TURN_SPEED;
            if (currentTurnSpeed < -MAX_TURN_SPEED) currentTurnSpeed = -MAX_TURN_SPEED;
            playerAngle += currentTurnSpeed * deltaTime;

            bool isMovingInput = false;
            if (receiver.IsKeyDown(KEY_KEY_W)) {
                currentSpeed += ACCELERATION * deltaTime;
                isMovingInput = true;
            }
            if (receiver.IsKeyDown(KEY_KEY_S)) {
                currentSpeed -= ACCELERATION * deltaTime;
                isMovingInput = true;
            }
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
            if (currentSpeed > MAX_SPEED) currentSpeed = MAX_SPEED;
            if (currentSpeed < -MAX_SPEED) currentSpeed = -MAX_SPEED;

            if (abs(currentSpeed) > 0.001f) {
                float angleRad = playerAngle * core::DEGTORAD;
                vector3df forwardVec(sin(angleRad), 0, cos(angleRad)); 
                vector3df proposedMove = forwardVec * currentSpeed * deltaTime;
                vector3df currentPos = playerNode->getPosition();
                vector3df targetPos = currentPos + proposedMove;
                vector3df clampedPos = navMesh->getClosestPointOnNavmesh(targetPos);
                playerNode->setPosition(clampedPos);
            }

            vector3df camPos = playerNode->getPosition();
            camPos.Y += eyeHeight;
            camera->setPosition(camPos);
            float angleRad = playerAngle * core::DEGTORAD;
            vector3df lookDir(sin(angleRad), 0, cos(angleRad)); 
            camera->setTarget(camPos + lookDir);
        }

        /*=========================================================
        RENDER PIPELINE
        =========================================================*/
        
        // PASS 1: Render Scene to Low-Res Texture (320x240)
        driver->setRenderTarget(rtt, true, true, SColor(255, 0, 0, 0));
        smgr->drawAll(); 

        // PASS 2: Render Texture to Screen using Shader
        driver->setRenderTarget(0, true, true, SColor(255, 0, 0, 0));
        
        driver->setMaterial(quadMaterial);
        driver->setTransform(video::ETS_WORLD, core::matrix4());
        driver->setTransform(video::ETS_VIEW, core::matrix4());
        driver->setTransform(video::ETS_PROJECTION, core::matrix4());
        
        driver->drawVertexPrimitiveList(quadVertices, 4, quadIndices, 2, video::EVT_STANDARD, scene::EPT_TRIANGLES, video::EIT_16BIT);

        driver->endScene();
    }

    if (navMesh) navMesh->drop();
    device->drop();

    return 0;
}