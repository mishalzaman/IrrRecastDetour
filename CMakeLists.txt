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
if(NOT EXISTS "${IRRLICHT_INCLUDE_DIR}")
    message(FATAL_ERROR "Irrlicht include directory not found at: ${IRRLICHT_INCLUDE_DIR}")
endif()
if(NOT EXISTS "${IRRLICHT_LIBRARY_FILE}")
    message(FATAL_ERROR "Irrlicht library not found at: ${IRRLICHT_LIBRARY_FILE}")
endif()
if(WIN32 AND NOT EXISTS "${IRRLICHT_DLL_FILE}")
    message(FATAL_ERROR "Irrlicht DLL not found at: ${IRRLICHT_DLL_FILE}")
endif()

# --- Configure Recast Navigation build options ---
set(RECASTNAVIGATION_DEMO OFF CACHE BOOL "Build demo" FORCE)
set(RECASTNAVIGATION_TESTS OFF CACHE BOOL "Build tests" FORCE)
set(RECASTNAVIGATION_EXAMPLES OFF CACHE BOOL "Build examples" FORCE)

# --- Add Recast Navigation subdirectory ---
if(EXISTS "${RECAST_ROOT}/CMakeLists.txt")
    # Build Recast/Detour as part of this project
    add_subdirectory(${RECAST_ROOT} ${CMAKE_BINARY_DIR}/recastnavigation)
    message(STATUS "Building Recast Navigation from source")
else()
    message(FATAL_ERROR "Recast Navigation not found at: ${RECAST_ROOT}")
endif()

# --- Define Core Library Files ---
# *** Config.h REMOVED from LIB_HEADERS ***
set(LIB_SOURCES
    "${CMAKE_SOURCE_DIR}/src/AbstractNavMesh.cpp"
    "${CMAKE_SOURCE_DIR}/src/StaticNavMesh.cpp"
    "${CMAKE_SOURCE_DIR}/src/TiledNavMesh.cpp"
)
set(LIB_HEADERS
    "${CMAKE_SOURCE_DIR}/src/AbstractNavMesh.h"
    "${CMAKE_SOURCE_DIR}/src/StaticNavMesh.h"
    "${CMAKE_SOURCE_DIR}/src/TiledNavMesh.h"
)

# --- Define Your NavMesh Library (STATIC) ---
add_library(IrrRecastDetour STATIC ${LIB_SOURCES} ${LIB_HEADERS})

# --- Link Recast/Detour TO your library ---
# We use PUBLIC so that any executable linking to IrrRecastDetour
# also links to these and gets their include paths.
target_link_libraries(IrrRecastDetour PUBLIC 
    Recast
    Detour
    DetourCrowd
    DetourTileCache
)

# --- Set include directories for your library ---
target_include_directories(IrrRecastDetour PUBLIC
    ${IRRLICHT_INCLUDE_DIR}
    "${CMAKE_SOURCE_DIR}/src"  # Demos still need this to find the lib headers
)

message(STATUS "Irrlicht include directory: ${IRRLICHT_INCLUDE_DIR}")
message(STATUS "Irrlicht library file: ${IRRLICHT_LIBRARY_FILE}")

# --- == Demos == ---

# --- Define files common to all demos ---
# *** Config.h ADDED to DEMO_COMMON_SOURCES ***
set(DEMO_COMMON_SOURCES
    "${CMAKE_SOURCE_DIR}/src/InputEventReceiver.cpp"
    "${CMAKE_SOURCE_DIR}/src/InputEventReceiver.h"
    "${CMAKE_SOURCE_DIR}/src/Config.h"
)

# --- Demo 1: Static NavMesh ---
add_executable(Demo_Static
    "${CMAKE_SOURCE_DIR}/src/Demo_Static.cpp"
    ${DEMO_COMMON_SOURCES}
)

# --- Demo 2: Tiled NavMesh ---
add_executable(Demo_Tiled
    "${CMAKE_SOURCE_DIR}/src/Demo_Tiled.cpp"
    ${DEMO_COMMON_SOURCES}
)

# --- Link Demos ---
set(DEMO_TARGETS
    Demo_Static
    Demo_Tiled
)

foreach(DEMO_TARGET ${DEMO_TARGETS})
    # Link our navmesh library
    target_link_libraries(${DEMO_TARGET} PRIVATE IrrRecastDetour)
    
    # Link Irrlicht
    target_link_libraries(${DEMO_TARGET} PRIVATE ${IRRLICHT_LIBRARY_FILE})
    
    # Additional Windows-specific libraries that Irrlicht depends on
    if(WIN32)
        target_link_libraries(${DEMO_TARGET} PRIVATE winmm.lib)
    endif()

    set_target_properties(${DEMO_TARGET} PROPERTIES
        VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${DEMO_TARGET}>"
    )

    # --- Post-Build Step (Copy the DLL on Windows) ---
    if(WIN32)
        add_custom_command(TARGET ${DEMO_TARGET} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${IRRLICHT_DLL_FILE}"
            $<TARGET_FILE_DIR:${DEMO_TARGET}>
            COMMENT "Copying Irrlicht.dll to ${DEMO_TARGET} directory"
        )
    endif()

    # --- Post-Build Step (Copy runtime assets) ---
    set(ASSETS_DEST_DIR "$<TARGET_FILE_DIR:${DEMO_TARGET}>/assets")
    set(ASSETS_SOURCE_DIR "${CMAKE_SOURCE_DIR}/assets")
    
    if(EXISTS "${ASSETS_SOURCE_DIR}")
        add_custom_command(TARGET ${DEMO_TARGET} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different
            "${ASSETS_SOURCE_DIR}"  # <-- Source
            "${ASSETS_DEST_DIR}"    # <-- Destination
            COMMENT "Copying 'assets' directory to ${DEMO_TARGET} directory"
        )
    else()
        message(WARNING "assets directory not found at: ${ASSETS_SOURCE_DIR}")
    endif()
endforeach()


# --- Organize files in Visual Studio ---
# *** Config.h moved from "Library" to "Demos" group ***
source_group("Library" FILES ${LIB_SOURCES} ${LIB_HEADERS})
source_group("Demos" FILES ${DEMO_COMMON_SOURCES} 
                            "${CMAKE_SOURCE_DIR}/src/Demo_Static.cpp" 
                            "${CMAKE_SOURCE_DIR}/src/Demo_Tiled.cpp")