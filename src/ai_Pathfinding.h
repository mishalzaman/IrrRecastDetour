#ifndef AI_PATHFINDING_H
#define AI_PATHFINDING_H

#include <irrlicht.h>
#include <vector>
#include "ai_util_RecastDetour.h"

using namespace irr;

class ai_Pathfinding
{
public:
	ai_Pathfinding();
	~ai_Pathfinding();

	bool load(
		scene::IMeshSceneNode* levelNode,
		scene::ISceneManager* smgr
	);

	std::vector<irr::core::vector3df> getPath(
		core::vector3df start,
		core::vector3df end
	);

	void renderDebugPath(
		std::vector<irr::core::vector3df> path,
		video::IVideoDriver* driver
	);
private:
	scene::ISceneNode* naviDebugData = 0;		// the nav mesh with polygon debug data
	ai_util_RecastDetour* recast = 0;
	std::vector<irr::core::vector3df> path;
};

#endif