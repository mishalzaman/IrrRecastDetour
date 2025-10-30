#include "ai_Pathfinding.h"

ai_Pathfinding::ai_Pathfinding()
{
}

ai_Pathfinding::~ai_Pathfinding()
{
    delete(recast);
}

bool ai_Pathfinding::load(scene::IMeshSceneNode* levelNode, scene::ISceneManager* smgr)
{
    this->recast = new ai_util_RecastDetour();

    if (levelNode)
    {
        if (recast->handleBuild(levelNode))
        {
            printf("PASSED::recast->handleBuild(levelNode)\n");
            scene::SMesh* smesh = new scene::SMesh();
            if (!recast->setupIrrSMeshFromRecastDetailMesh(smesh))
            {
                printf("recast->setupIrrSMeshFromRecastDetailMesh(smesh): FAILED!\n");
                return false;
            }
            else
            {
                printf("recast->setupIrrSMeshFromRecastDetailMesh(smesh): PASSED!\n");
                this->naviDebugData = smgr->addOctreeSceneNode(smesh);
                this->naviDebugData->setName("Terrain");
                this->naviDebugData->setDebugDataVisible(scene::EDS_MESH_WIRE_OVERLAY);
                this->naviDebugData->setPosition(core::vector3df(0, -1, 0));
            }
            smesh->drop();
        }
        else {
            printf("ERROR::recast->handleBuild(levelNode)\n");
            return false;
        }
    }
    else {
        printf("ERROR::levelNode is null\n");
        return false;
    }

    return true;
}

std::vector<irr::core::vector3df> ai_Pathfinding::getPath(core::vector3df start, core::vector3df end)
{
    return this->recast->returnPath(start, end);
}

void ai_Pathfinding::renderDebugPath(std::vector<irr::core::vector3df> path, video::IVideoDriver* driver)
{
    for (int i = 0; i < path.size(); i++) {
        if ((i + 1) == (int)path.size()) {
            break;
        }
        video::SMaterial m;
        m.Lighting = false;
        driver->setMaterial(m);
        driver->setTransform(video::ETS_WORLD, core::matrix4());
        core::vector3df n1 = core::vector3df(path[i].X, path[i].Y + 0.1, path[i].Z);
        core::vector3df n2 = core::vector3df(path[i + 1].X, path[i + 1].Y + 0.1, path[i + 1].Z);
        driver->draw3DLine(n1, n2, video::SColor(255, 0, 0, 255));
    };
}