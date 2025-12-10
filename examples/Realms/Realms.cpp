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

enum
{
    ID_IsNotPickable = 0,
    IDFlag_IsPickable = 1 << 0,
    IDFlag_IsHighlightable = 1 << 1
};

/*===========================================================
HELPERS
===========================================================*/
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
    // Set stencil buffer to false as we are not using any shadow methods
    IrrlichtDevice* device = createDevice(
        video::EDT_OPENGL,
        dimension2d<u32>(Config::WINDOW_WIDTH, Config::WINDOW_HEIGHT),
        32, false, false, false, &receiver); 

    if (!device) {
        std::cerr << "Failed to create Irrlicht device!" << std::endl;
        return 1;
    }
    // Updated window caption to reflect the change
    device->setWindowCaption(L"Irrlicht Recast/Detour Demo - Basic Phong Lighting (No Shadows)");

    IVideoDriver* driver = device->getVideoDriver();
    ISceneManager* smgr = device->getSceneManager();
    smgr->setAmbientLight(SColorf(0.3f, 0.3f, 0.3f, 1.0f)); // Set a basic ambient light

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
    DIRECTIONAL LIGHT
    =========================================================*/
    // Simple directional light setup
    scene::ILightSceneNode* light = smgr->addLightSceneNode(0, vector3df(50.0f, 500.0f, 50.0f),
        SColorf(1.0f, 1.0f, 1.0f), 800.0f);
    light->setLightType(video::ELT_DIRECTIONAL);
    light->setRotation(vector3df(45, 0, 0)); 

    /*=========================================================
                           NAVMESH SETUP
     ----------------------------------------------------------
    * =========================================================*/

    /*=========================================================
    1. LOAD MAP
    =========================================================*/
    scene::ISceneCollisionManager* levelCollisionManager = nullptr;
    IAnimatedMesh* mapMesh = smgr->getMesh("assets/realms/realms.obj");
    if (!mapMesh) {
        std::cerr << "Failed to load level mesh: assets/realms/realms.obj" << std::endl;
        device->drop();
        return 1;
    }
    IMeshSceneNode* mapNode = smgr->addMeshSceneNode(mapMesh->getMesh(0));

    if (mapNode) {
        mapNode->setPosition(vector3df(0, 0, 0));
        mapNode->setID(IDFlag_IsPickable);
        mapNode->setVisible(true);
        mapNode->setMaterialFlag(EMF_NORMALIZE_NORMALS, true); // Recommended for lighting
        
        std::cout << "Map node material count: " << mapNode->getMaterialCount() << std::endl;

        // Set material to a standard lighting material
        s32 materialType = video::EMT_SOLID; 
        for (u32 i = 0; i < mapNode->getMaterialCount(); i++)
        {
            SMaterial& mat = mapNode->getMaterial(i);
            mat.MaterialType = (E_MATERIAL_TYPE)materialType;
            mat.Lighting = true; // Enable lighting
            mat.Wireframe = false;
        }

        std::cout << "Map node material type set to: " << materialType << " with lighting enabled." << std::endl;

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
    2. CREATE AND BUILD NAVMESH
    =========================================================*/
	// a. Initialize StaticNavMesh and.
    CStaticNavMesh* navMesh = new CStaticNavMesh(smgr->getRootSceneNode(), smgr);

    // b. Create a NavMeshParams struct. 
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

	// c. Build the navmesh from your mesh node and NavMeshParams parameters.
    bool success = navMesh->build(mapNode, params);

    if (!success) {
        std::cerr << "Initial navmesh build failed!" << std::endl;
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
    playerNode->setMaterialFlag(EMF_LIGHTING, true); // Player needs lighting enabled
    playerNode->setMaterialFlag(EMF_NORMALIZE_NORMALS, true); // Recommended for lighting
    playerNode->setMaterialFlag(EMF_GOURAUD_SHADING, true); 

    // REMOVED: Shadow casting code is removed for compatibility.

    playerNode->getMaterial(0).EmissiveColor.set(0, 0, 0, 0); 
    playerNode->getMaterial(0).AmbientColor.set(255, 255, 0, 0); // Set red ambient color
    playerNode->getMaterial(0).DiffuseColor.set(255, 255, 0, 0); // Set red diffuse color

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

            // CORRECTED LINE: Removed the extra ->getSceneCollisionManager() call
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
        driver->beginScene(true, true, SColor(255, 100, 100, 100)); 

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