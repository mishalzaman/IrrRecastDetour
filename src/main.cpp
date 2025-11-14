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

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

#ifdef _IRR_WINDOWS_
#pragma comment(lib, "Irrlicht.lib")
#endif


int main() {

    return 0;
}