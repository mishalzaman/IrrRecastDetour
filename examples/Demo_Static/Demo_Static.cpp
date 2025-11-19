#include <irrlicht.h>
#include <iostream>
#include <vector>
#include <IrrRecastDetour/StaticNavMesh.h>

#include "../common/InputEventReceiver.h"
#include "../common/Config.h"
#include "../common/NavMeshGUI.h"

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

// Vertex Shader (Unreal Engine style)
const char* vertexShaderCode = R"(
#version 330 core

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inTexCoord;

uniform mat4 mWorldViewProj;
uniform mat4 mWorld;
uniform vec3 mLightPos;
uniform vec3 mCameraPos;

out vec3 fragPos;
out vec3 fragNormal;
out vec2 fragTexCoord;
out vec3 viewDir;
out vec3 lightDir;

void main()
{
    vec4 worldPos = mWorld * vec4(inPosition, 1.0);
    fragPos = worldPos.xyz;
    fragNormal = normalize(mat3(mWorld) * inNormal);
    fragTexCoord = inTexCoord;
    
    viewDir = normalize(mCameraPos - fragPos);
    lightDir = normalize(mLightPos - fragPos);
    
    gl_Position = mWorldViewProj * vec4(inPosition, 1.0);
}
)";

// Fragment Shader (Unreal Engine PBR-style)
const char* fragmentShaderCode = R"(
#version 330 core

in vec3 fragPos;
in vec3 fragNormal;
in vec2 fragTexCoord;
in vec3 viewDir;
in vec3 lightDir;

uniform vec3 mLightColor;
uniform vec3 mAmbientColor;
uniform vec4 mBaseColor;
uniform float mMetallic;
uniform float mRoughness;
uniform float mAmbientOcclusion;

out vec4 FragColor;

const float PI = 3.14159265359;

// Fresnel-Schlick approximation
vec3 fresnelSchlick(float cosTheta, vec3 F0)
{
    return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
}

// GGX/Trowbridge-Reitz normal distribution
float distributionGGX(vec3 N, vec3 H, float roughness)
{
    float a = roughness * roughness;
    float a2 = a * a;
    float NdotH = max(dot(N, H), 0.0);
    float NdotH2 = NdotH * NdotH;
    
    float num = a2;
    float denom = (NdotH2 * (a2 - 1.0) + 1.0);
    denom = PI * denom * denom;
    
    return num / denom;
}

// Smith's method with GGX
float geometrySchlickGGX(float NdotV, float roughness)
{
    float r = (roughness + 1.0);
    float k = (r * r) / 8.0;
    
    float num = NdotV;
    float denom = NdotV * (1.0 - k) + k;
    
    return num / denom;
}

float geometrySmith(vec3 N, vec3 V, vec3 L, float roughness)
{
    float NdotV = max(dot(N, V), 0.0);
    float NdotL = max(dot(N, L), 0.0);
    float ggx2 = geometrySchlickGGX(NdotV, roughness);
    float ggx1 = geometrySchlickGGX(NdotL, roughness);
    
    return ggx1 * ggx2;
}

void main()
{
    vec3 N = normalize(fragNormal);
    vec3 V = normalize(viewDir);
    vec3 L = normalize(lightDir);
    vec3 H = normalize(V + L);
    
    // Base reflectivity (F0)
    vec3 F0 = vec3(0.04);
    F0 = mix(F0, mBaseColor.rgb, mMetallic);
    
    // Calculate radiance
    float distance = length(lightDir);
    float attenuation = 1.0 / (distance * distance * 0.01 + 1.0);
    vec3 radiance = mLightColor * attenuation;
    
    // Cook-Torrance BRDF
    float NDF = distributionGGX(N, H, mRoughness);
    float G = geometrySmith(N, V, L, mRoughness);
    vec3 F = fresnelSchlick(max(dot(H, V), 0.0), F0);
    
    vec3 kS = F;
    vec3 kD = vec3(1.0) - kS;
    kD *= 1.0 - mMetallic;
    
    vec3 numerator = NDF * G * F;
    float denominator = 4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0) + 0.001;
    vec3 specular = numerator / denominator;
    
    // Add to outgoing radiance
    float NdotL = max(dot(N, L), 0.0);
    vec3 Lo = (kD * mBaseColor.rgb / PI + specular) * radiance * NdotL;
    
    // Ambient lighting with AO
    vec3 ambient = mAmbientColor * mBaseColor.rgb * mAmbientOcclusion;
    
    vec3 color = ambient + Lo;
    
    // HDR tonemapping (Unreal uses ACES, this is Reinhard for simplicity)
    color = color / (color + vec3(1.0));
    
    // Gamma correction
    color = pow(color, vec3(1.0/2.2));
    
    FragColor = vec4(color, mBaseColor.a);
}
)";

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
        dimension2d<u32>(Config::WINDOW_WIDTH, Config::WINDOW_HEIGHT),
        32, false, false, false, &receiver);

    if (!device) {
        std::cerr << "Failed to create Irrlicht device!" << std::endl;
        return 1;
    }
    device->setWindowCaption(L"Irrlicht Recast/Detour Demo - Unreal-Style Lighting");

    IVideoDriver* driver = device->getVideoDriver();
    ISceneManager* smgr = device->getSceneManager();
    IGUIEnvironment* guienv = device->getGUIEnvironment();

    /*=========================================================
    CUSTOM SHADER SETUP
    =========================================================*/
    s32 materialType = 0;

    if (driver->queryFeature(EVDF_PIXEL_SHADER_3_0) && driver->queryFeature(EVDF_VERTEX_SHADER_3_0))
    {
        IGPUProgrammingServices* gpu = driver->getGPUProgrammingServices();

        if (gpu)
        {
            materialType = gpu->addHighLevelShaderMaterial(
                vertexShaderCode, "main", EVST_VS_3_0,
                fragmentShaderCode, "main", EPST_PS_3_0,
                nullptr, EMT_SOLID);

            if (materialType == -1)
            {
                std::cerr << "Failed to create custom shader material!" << std::endl;
                materialType = EMT_SOLID; // Fallback to solid material
            }
            else
            {
                std::cout << "Custom Unreal-style shader loaded successfully!" << std::endl;
            }
        }
    }
    else
    {
        std::cerr << "Shader 3.0 not supported, using default material" << std::endl;
        materialType = EMT_SOLID;
    }

    /*=========================================================
    LOAD MAP
    =========================================================*/
    scene::ISceneCollisionManager* levelCollisionManager = nullptr;
    IAnimatedMesh* mapMesh = smgr->getMesh("assets/test_level/test_level.obj");
    if (!mapMesh) {
        std::cerr << "Failed to load level mesh: assets/test_level/test_level.obj" << std::endl;
        device->drop();
        return 1;
    }
    IMeshSceneNode* mapNode = smgr->addMeshSceneNode(mapMesh->getMesh(0));

    if (mapNode) {
        mapNode->setMaterialType((E_MATERIAL_TYPE)materialType);
        mapNode->setMaterialFlag(EMF_LIGHTING, true); // Enable lighting for shader
        mapNode->setMaterialFlag(EMF_WIREFRAME, false);
        mapNode->setPosition(vector3df(0, 0, 0));
        mapNode->setID(IDFlag_IsPickable);
        mapNode->setVisible(true);

        // Set PBR material properties
        for (u32 i = 0; i < mapNode->getMaterialCount(); i++)
        {
            SMaterial& mat = mapNode->getMaterial(i);
            mat.DiffuseColor.set(255, 180, 180, 180); // Base color (light gray)
            mat.AmbientColor.set(255, 60, 60, 70);    // Ambient color (cool ambient)
            mat.SpecularColor.set(255, 255, 255, 255); // Specular
            mat.Shininess = 32.0f;
        }

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
    LIGHTING SETUP (Unreal-style)
    =========================================================*/
    // Add a bright directional light (sun)
    ILightSceneNode* sunLight = smgr->addLightSceneNode(0, vector3df(100, 100, 100));
    sunLight->setLightType(ELT_POINT);

    SLight& lightData = sunLight->getLightData();
    lightData.DiffuseColor = SColorf(1.0f, 0.95f, 0.9f, 1.0f);  // Warm sunlight
    lightData.AmbientColor = SColorf(1.0f, 0.35f, 0.4f, 1.0f);  // Cool ambient (sky color)
    lightData.SpecularColor = SColorf(0.1f, 0.1f, 0.1f, 0.9f);
    lightData.Radius = 1000.0f;
    lightData.CastShadows = false;

    // Set global ambient light
    smgr->setAmbientLight(SColorf(0.2f, 0.22f, 0.25f));

    /*=========================================================
    BUILD NAVMESH
    =========================================================*/
    StaticNavMesh* navMesh = new StaticNavMesh(smgr->getRootSceneNode(), smgr);
    NavMeshParams params;
    params.AgentHeight = params.AgentRadius * 2;

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
        =========================================================*/
        //ISceneNode* debugNavMeshNode = navMesh->renderNavMesh();
        //if (debugNavMeshNode) {
        //    debugNavMeshNode->setMaterialFlag(EMF_LIGHTING, false);
        //    debugNavMeshNode->setMaterialFlag(EMF_WIREFRAME, true);
        //    debugNavMeshNode->getMaterial(0).EmissiveColor.set(255, 0, 150, 255); // Cyan-ish
        //}

        /*=========================================================
        PLAYER AGENT
        =========================================================*/
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
    GUI
    =========================================================*/
    NavMeshGUI* navMeshGui = new NavMeshGUI(guienv);
    navMeshGui->Load(windowWidth, windowHeight);

    receiver.setGUIEnvironment(guienv);
    receiver.setNavMeshGUI(navMeshGui);

    /*=========================================================
    MAIN LOOP
    =========================================================*/
    u32 then = device->getTimer()->getTime();

    while (device->run()) {
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

        guienv->drawAll();
        driver->endScene();
    }

    if (navMesh)
        navMesh->drop();

    device->drop();
    return 0;
}