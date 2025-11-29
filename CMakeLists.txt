cmake_minimum_required(VERSION 3.14)
project(IrrRecastDetour LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Include FetchContent Module ---
include(FetchContent)

# ==============================================================================
# 1. Fetch Irrlicht (Source Code)
# ==============================================================================
FetchContent_Declare(
    irrlicht
    GIT_REPOSITORY https://github.com/mishalzaman/irrlicht-1.8.5.git
    GIT_TAG        main
)

FetchContent_GetProperties(irrlicht)
if(NOT irrlicht_POPULATED)
    FetchContent_Populate(irrlicht)
endif()

# Set root paths
set(IRRLICHT_ROOT "${irrlicht_SOURCE_DIR}")
set(IRRLICHT_INCLUDE_DIR "${IRRLICHT_ROOT}/include")

# ==============================================================================
# 2. Compile Irrlicht (The Missing Step)
# ==============================================================================
# We need to find X11 and OpenGL to compile Irrlicht on Linux
if(UNIX AND NOT APPLE)
    find_package(X11 REQUIRED)
    find_package(OpenGL REQUIRED)
    
    # Define where the compiled file WILL be after we build it
    set(IRRLICHT_LIBRARY_FILE "${IRRLICHT_ROOT}/lib/Linux/libIrrlicht.a")

    # Command to run 'make' inside the Irrlicht source directory
    add_custom_command(
        OUTPUT "${IRRLICHT_LIBRARY_FILE}"
        COMMAND make NDEBUG=1
        WORKING_DIRECTORY "${IRRLICHT_ROOT}/source/Irrlicht"
        COMMENT "Compiling Irrlicht 1.8.5 from source... (This may take a minute)"
    )

    # Create a custom target that triggers the command above
    add_custom_target(IrrlichtMake DEPENDS "${IRRLICHT_LIBRARY_FILE}")

    # Create a library target that points to the file
    add_library(Irrlicht STATIC IMPORTED GLOBAL)
    add_dependencies(Irrlicht IrrlichtMake) # Force build before linking
    set_target_properties(Irrlicht PROPERTIES IMPORTED_LOCATION "${IRRLICHT_LIBRARY_FILE}")

elseif(WIN32)
    # On Windows, compiling 1.8.5 via command line is harder (needs MSBuild).
    # For now, we assume standard paths, but ideally you should commit .lib files 
    # to your repo or upgrade to a CMake-supported Irrlicht fork.
    set(IRRLICHT_LIBRARY_FILE "${IRRLICHT_ROOT}/lib/Win64-visualStudio/Irrlicht.lib")
    add_library(Irrlicht STATIC IMPORTED GLOBAL)
    set_target_properties(Irrlicht PROPERTIES IMPORTED_LOCATION "${IRRLICHT_LIBRARY_FILE}")
endif()

# ==============================================================================
# 3. Fetch and Configure RecastNavigation
# ==============================================================================
set(RECASTNAVIGATION_DEMO OFF CACHE BOOL "Build demo" FORCE)
set(RECASTNAVIGATION_TESTS OFF CACHE BOOL "Build tests" FORCE)
set(RECASTNAVIGATION_EXAMPLES OFF CACHE BOOL "Build examples" FORCE)

FetchContent_Declare(
    recast
    GIT_REPOSITORY https://github.com/recastnavigation/recastnavigation.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(recast)
set(RECAST_ROOT "${recast_SOURCE_DIR}")

# ==============================================================================
# 4. Define Core Library (IrrRecastDetour)
# ==============================================================================
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

add_library(IrrRecastDetour STATIC ${LIB_SOURCES} ${LIB_HEADERS})

# Link libraries
# Note: We now link against the target 'Irrlicht' defined above
target_link_libraries(IrrRecastDetour PUBLIC 
    Irrlicht 
    Recast
    Detour
    DetourCrowd
    DetourTileCache
)

# On Linux, we also need to link X11/GL libraries to the final app
if(UNIX AND NOT APPLE)
    target_link_libraries(IrrRecastDetour PUBLIC 
        ${X11_LIBRARIES} 
        ${X11_Xxf86vm_LIB} 
        ${OPENGL_LIBRARIES}
    )
endif()

target_include_directories(IrrRecastDetour PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    ${IRRLICHT_INCLUDE_DIR}
)

# --- Add the examples/ folder ---
add_subdirectory(examples)