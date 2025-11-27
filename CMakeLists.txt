cmake_minimum_required(VERSION 3.10)
project(IrrRecastDetour LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Define Irrlicht Paths ---
set(IRRLICHT_ROOT "${CMAKE_SOURCE_DIR}/irrlicht-1.8.5")
set(IRRLICHT_INCLUDE_DIR "${IRRLICHT_ROOT}/include")

# --- Define Recast Navigation Paths ---
set(RECAST_ROOT "${CMAKE_SOURCE_DIR}/recastnavigation")

# --- Platform-Specific Library Paths ---
# (This whole section remains unchanged)
if(WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        # 64-bit Windows
        set(IRRLICHT_LIBRARY_FILE "${IRRLICHT_ROOT}/lib/Win64-visualStudio/Irrlicht.lib")
        set(IRRLICHT_DLL_FILE "${IRRLICHT_ROOT}/bin/Win64-visualStudio/Irrlicht.dll")
    else()
        # 32-bit Windows
        set(IRRLICHT_LIBRARY_FILE "${IRRLICHT_ROOT}/lib/Win32-visualStudio/Irrlicht.lib")
        set(IRRLICHT_DLL_FILE "${IRRLICHT_ROOT}/bin/Win32-visualStudio/Irrlicht.dll")
    endif()
elseif(UNIX AND NOT APPLE)
    # Linux
    set(IRRLICHT_LIBRARY_FILE "${IRRLICHT_ROOT}/lib/Linux/libIrrlicht.a")
elseif(APPLE)
    # macOS
    set(IRRLICHT_LIBRARY_FILE "${IRRLICHT_ROOT}/lib/MacOSX/libIrrlicht.a")
endif()

# --- Validate Paths ---
# (This whole section remains unchanged)
if(NOT EXISTS "${IRRLICHT_INCLUDE_DIR}")
    message(FATAL_ERROR "Irrlicht include directory not found at: ${IRRLICHT_INCLUDE_DIR}")
endif()
# ... (rest of path validation) ...

# --- Configure Recast Navigation build options ---
# (This whole section remains unchanged)
set(RECASTNAVIGATION_DEMO OFF CACHE BOOL "Build demo" FORCE)
# ... (rest of Recast options) ...

# --- Add Recast Navigation subdirectory ---
# (This whole section remains unchanged)
if(EXISTS "${RECAST_ROOT}/CMakeLists.txt")
    add_subdirectory(${RECAST_ROOT} ${CMAKE_BINARY_DIR}/recastnavigation)
else()
    message(FATAL_ERROR "Recast Navigation not found at: ${RECAST_ROOT}")
endif()

# --- Define Core Library Files ---
# *** NEW PATHS ***
set(LIB_SOURCES
    "src/INavMesh.cpp"
    "src/CStaticNavMesh.cpp"
    "src/CTiledNavMesh.cpp"
)
set(LIB_HEADERS
    "include/IrrRecastDetour/INavMesh.h"
    "include/IrrRecastDetour/CStaticNavMesh.h"
    "include/IrrRecastDetour/CTiledNavMesh.h"
)

# --- Define Your NavMesh Library (STATIC) ---
add_library(IrrRecastDetour STATIC ${LIB_SOURCES} ${LIB_HEADERS})

# --- Link Recast/Detour TO your library ---
target_link_libraries(IrrRecastDetour PUBLIC 
    Recast
    Detour
    DetourCrowd
    DetourTileCache
)

# --- Set include directories for your library ---
# *** NEW: This makes the "include" folder public ***
target_include_directories(IrrRecastDetour PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    ${IRRLICHT_INCLUDE_DIR}
)

# --- Organize files in Visual Studio ---
source_group("Library Headers" FILES ${LIB_HEADERS})
source_group("Library Sources" FILES ${LIB_SOURCES})

# --- Add the examples/ folder ---
# *** NEW: This tells CMake to look for the new CMakeLists.txt ***
add_subdirectory(examples)