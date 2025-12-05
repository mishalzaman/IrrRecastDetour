#include <irrlicht.h>
#include <iostream>
#include <vector>
#include <IrrRecastDetour/CTiledNavMesh.h>

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

enum
{
    ID_IsNotPickable = 0,
    IDFlag_IsPickable = 1 << 0,
    IDFlag_IsHighlightable = 1 << 1
};

std::string readFile(const std::string& filePath) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Could not open file: " << filePath << std::endl;
        return "";
    }
    else {
        std::cout << "Successfully opened file: " << filePath << std::endl;
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

/**
 * @brief Finds the 3D world position of a mouse click on the level geometry.
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

    const ISceneNode* node = hitNode;
    while (node)
    {
        if (node == mapNode)
        {
            return true;
        }
        node = node->getParent();
    }

    return false;
}

int main() {
    /*=========================================================
    IRRLICHT SETUP
    =========================================================*/
    InputEventListener receiver;
    IrrlichtDevice* device = createDevice(
        video::EDT_OPENGL,
        dimension2d<u32>(windowWidth, windowHeight),
        32, false, false, false, &receiver);

    if (!device) {
        std::cerr << "Failed to create Irrlicht device!" << std::endl;
        return 1;
    }
    device->setWindowCaption(L"Irrlicht Recast/Detour Demo - Tiled NavMesh");

    IVideoDriver* driver = device->getVideoDriver();
    ISceneManager* smgr = device->getSceneManager();

    /*=========================================================
    CAMERA - PERSPECTIVE WITH ORBITAL CONTROL
    =========================================================*/
    float cameraDistance = 15.0f;
    float cameraAngleH = 0.0f;
    float cameraAngleV = 45.0f;
    const float cameraRotationSpeed = 0.3f;

    ICameraSceneNode* camera = smgr->addCameraSceneNode();

    const f32 aspectRatio = (f32)windowWidth / (f32)windowHeight;
    camera->setAspectRatio(aspectRatio);
    camera->setFOV(core::degToRad(60.0f));
    camera->setNearValue(0.1f);
    camera->setFarValue(1000.0f);

    camera->setPosition(vector3df(0, 15, 0));
    camera->setTarget(vector3df(0, 0, 0));

    /*=========================================================
    CUSTOM SHADER CALLBACK
    =========================================================*/
    class ShaderCallback : public video::IShaderConstantSetCallBack
    {
    private:
        ISceneManager* smgr;

    public:
        ShaderCallback(ISceneManager* sceneManager) : smgr(sceneManager) {}

        virtual void OnSetConstants(video::IMaterialRendererServices* services, s32 userData)
        {
            video::IVideoDriver* driver = services->getVideoDriver();

            // 1. MATRICES (Correct - These belong in Vertex Shader)
            core::matrix4 worldViewProj = driver->getTransform(video::ETS_PROJECTION);
            worldViewProj *= driver->getTransform(video::ETS_VIEW);
            worldViewProj *= driver->getTransform(video::ETS_WORLD);
            services->setVertexShaderConstant("mWorldViewProj", worldViewProj.pointer(), 16);

            core::matrix4 world = driver->getTransform(video::ETS_WORLD);
            services->setVertexShaderConstant("mWorld", world.pointer(), 16);

            // 2. LIGHT POSITION (CHANGE THIS)
            f32 lightPos[3] = { 50.0f, 500.0f, 50.0f };
            services->setPixelShaderConstant("mLightPos", lightPos, 3);

            f32 ambientStrength = 0.2f;
            services->setPixelShaderConstant("mAmbientStrength", &ambientStrength, 1);

            f32 specularStrength = 0.1f;
            services->setPixelShaderConstant("mSpecularStrength", &specularStrength, 1);

            int textureUnit = 0;
            services->setPixelShaderConstant("mTexture", &textureUnit, 1);
        }
    };

    /*=========================================================
    CUSTOM SHADER SETUP
    =========================================================*/
    ShaderCallback* shaderCallback = new ShaderCallback(smgr);

    s32 materialType = 0;
    std::string vertCode = readFile("assets/main.vert");
    std::string fragCode = readFile("assets/main.frag");

    if (vertCode.empty() || fragCode.empty())
    {
        std::cerr << "Failed to read shader files!" << std::endl;
        materialType = EMT_SOLID;
    }
    else if (driver->queryFeature(video::EVDF_ARB_GLSL))
    {
        IGPUProgrammingServices* gpu = driver->getGPUProgrammingServices();

        if (gpu)
        {
            materialType = gpu->addHighLevelShaderMaterial(
                vertCode.c_str(), "main", video::EVST_VS_1_1,
                fragCode.c_str(), "main", video::EPST_PS_1_1,
                shaderCallback, video::EMT_SOLID, 0, video::EGSL_DEFAULT);

            if (materialType == -1)
            {
                std::cerr << "Failed to create custom shader material!" << std::endl;
                materialType = EMT_SOLID;
            }
            else
            {
                std::cout << "Custom Unreal-style shader loaded successfully! Material type: " << materialType << std::endl;
            }
        }
    }
    else
    {
        std::cerr << "GLSL not supported, using default material" << std::endl;
        materialType = EMT_SOLID;
    }

    /*=========================================================
                           NAVMESH SETUP
     ----------------------------------------------------------
    * =========================================================*/

    /*=========================================================
    1. LOAD MAP
    =========================================================*/
    scene::ISceneCollisionManager* levelCollisionManager = nullptr;
    IAnimatedMesh* mapMesh = smgr->getMesh("assets/demo/demo.obj");
    if (!mapMesh) {
        std::cerr << "Failed to load level mesh: assets/demo/demo.obj" << std::endl;
        device->drop();
        return 1;
    }
    IMeshSceneNode* mapNode = smgr->addMeshSceneNode(mapMesh->getMesh(0));

    if (mapNode) {
        mapNode->setPosition(vector3df(0, 0, 0));
        mapNode->setID(IDFlag_IsPickable);
        mapNode->setVisible(true);

        std::cout << "Map node material count: " << mapNode->getMaterialCount() << std::endl;

        for (u32 i = 0; i < mapNode->getMaterialCount(); i++)
        {
            SMaterial& mat = mapNode->getMaterial(i);
            mat.MaterialType = (E_MATERIAL_TYPE)materialType;
            mat.Lighting = false;
            mat.Wireframe = false;
        }

        std::cout << "Map node material type set to: " << materialType << std::endl;

        scene::ITriangleSelector* selector = smgr->createOctreeTriangleSelector(
            mapNode->getMesh(), mapNode, 128
        );
        if (selector) {
            mapNode->setTriangleSelector(selector);
            selector->drop();
            levelCollisionManager = smgr->getSceneCollisionManager();
            std::cout << "Triangle selector set successfully." << std::endl;
        }
    }

    /*=========================================================
    2. CREATE AND BUILD TILED NAVMESH
    =========================================================*/
    // a. Initialize TiledNavMesh
    CTiledNavMesh* navMesh = new CTiledNavMesh(smgr->getRootSceneNode(), smgr);

    // b. Create a NavMeshParams struct. 
    //    You can change the parameters to suit the map and agent requirements. 
    NavMeshParams params;
    params.CellSize = 0.15f;
    params.CellHeight = 0.2f;
    params.AgentHeight = 0.8f;
    params.AgentRadius = 0.4f;
    params.AgentMaxClimb = 0.6f;
    params.AgentMaxSlope = 45.f;
    params.RegionMinSize = 8.f;
    params.RegionMergeSize = 20.f;
    params.EdgeMaxError = 1.3f;
    params.EdgeMaxLen = 12.f;
    params.VertsPerPoly = 6;
    params.DetailSampleDist = 6.0f;
    params.DetailSampleMaxError = 1.0f;
    params.KeepInterResults = true; // Required for debug visualization

    // c. Define tile size (in cells)
    const int tileSize = 32;

    // d. Build the tiled navmesh from your mesh node, NavMeshParams, and tile size
    bool success = navMesh->build(mapNode, params, tileSize);

    if (!success) {
        std::cerr << "Initial tiled navmesh build failed!" << std::endl;
        return 1;
    }

    /*=========================================================
    2.a RENDER NAVMESH (OPTIONAL DEBUG VISUALIZATION)
    =========================================================*/

    ISceneNode* debugNavMeshNode = nullptr;
    debugNavMeshNode = navMesh->renderNavMesh();
    if (debugNavMeshNode) {
        debugNavMeshNode->setMaterialFlag(EMF_LIGHTING, false);
        debugNavMeshNode->setMaterialFlag(EMF_WIREFRAME, true);
        debugNavMeshNode->getMaterial(0).EmissiveColor.set(255, 0, 150, 255); // Cyan-ish
    }

    /*=========================================================
    3. ADD PLAYER AGENT
    =========================================================*/
    ISceneNode* playerNode = nullptr;
    int playerId = -1;
    vector3df initialPlayerPos(5, 1, 5);

    playerNode = smgr->addSphereSceneNode(params.AgentRadius);
    playerNode->setMaterialFlag(EMF_LIGHTING, false);
    playerNode->getMaterial(0).EmissiveColor.set(255, 255, 0, 0); // Red
    playerNode->setPosition(initialPlayerPos);
    playerId = navMesh->addAgent(playerNode, params.AgentRadius, params.AgentHeight);

    /*=========================================================
    MAIN LOOP
    =========================================================*/
    u32 then = device->getTimer()->getTime();

    while (device->run()) {
        u32 now = device->getTimer()->getTime();
        float deltaTime = (float)(now - then) / 1000.0f;
        then = now;

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

            cameraAngleH -= dragDelta.X * cameraRotationSpeed;
            cameraAngleV -= dragDelta.Y * cameraRotationSpeed;

            if (cameraAngleV < 5.0f) cameraAngleV = 5.0f;
            if (cameraAngleV > 89.0f) cameraAngleV = 89.0f;
        }

        /*--------------------------------------------------------
        LEFT-CLICK MOVE PLAYER
        --------------------------------------------------------*/
        if (success && receiver.wasMouseClicked()) {
            position2di mousePos = receiver.getMousePos();

            core::line3d<f32> ray = smgr->getSceneCollisionManager()->getRayFromScreenCoordinates(
                mousePos, camera);

            core::vector3df intersectionPoint;
            core::triangle3df hitTriangle;

            scene::ISceneNode* selectedSceneNode =
                levelCollisionManager->getSceneNodeAndCollisionPointFromRay(
                    ray, intersectionPoint, hitTriangle, IDFlag_IsPickable, 0);

            if (selectedSceneNode && selectedSceneNode == mapNode) {
                std::cout << "Mouse clicked mesh at: "
                    << intersectionPoint.X << ", "
                    << intersectionPoint.Y << ", "
                    << intersectionPoint.Z << std::endl;

                if (playerId != -1) {
                    navMesh->setAgentTarget(playerId, intersectionPoint);
                }
            }
        }

        /*--------------------------------------------------------
        UPDATE SHADER UNIFORMS
        --------------------------------------------------------*/
        // Shader uniforms are automatically handled by Irrlicht
        // through the material properties and light settings

        /*--------------------------------------------------------
        CAMERA FOLLOW PLAYER
        --------------------------------------------------------*/
        if (success && playerNode) {
            vector3df playerPos = playerNode->getPosition();

            float angleHRad = core::degToRad(cameraAngleH);
            float angleVRad = core::degToRad(cameraAngleV);

            float x = playerPos.X + cameraDistance * sin(angleVRad) * cos(angleHRad);
            float y = playerPos.Y + cameraDistance * cos(angleVRad);
            float z = playerPos.Z + cameraDistance * sin(angleVRad) * sin(angleHRad);

            camera->setPosition(vector3df(x, y, z));
            camera->setTarget(playerPos);
        }

        // --- Render ---
        driver->beginScene(true, true, SColor(255, 30, 35, 45)); // Darker, cooler background

        smgr->drawAll();

        if (success) {
            navMesh->renderAgentPaths(driver);
        }

        driver->endScene();
    }

    if (navMesh)
        navMesh->drop();

    device->drop();

    return 0;
}
