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

# --- Auto-discover all source files ---
file(GLOB_RECURSE SOURCE_FILES 
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/*.c"
)

file(GLOB_RECURSE HEADER_FILES 
    "${CMAKE_SOURCE_DIR}/src/*.h"
    "${CMAKE_SOURCE_DIR}/src/*.hpp"
)

# --- Your Project Executable ---
add_executable(IrrRecastDetour ${SOURCE_FILES} ${HEADER_FILES})

set_target_properties(IrrRecastDetour PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:IrrRecastDetour>"
)

# --- Link Irrlicht ---
target_include_directories(IrrRecastDetour PRIVATE 
    ${IRRLICHT_INCLUDE_DIR}
    "${CMAKE_SOURCE_DIR}/src"
)

target_link_libraries(IrrRecastDetour PRIVATE 
    ${IRRLICHT_LIBRARY_FILE}
    Recast
    Detour
    DetourCrowd
    DetourTileCache
)

# Additional Windows-specific libraries that Irrlicht depends on
if(WIN32)
    target_link_libraries(IrrRecastDetour PRIVATE winmm.lib)
endif()

# Organize files in Visual Studio
source_group("Source Files" FILES ${SOURCE_FILES})
source_group("Header Files" FILES ${HEADER_FILES})

message(STATUS "Irrlicht include directory: ${IRRLICHT_INCLUDE_DIR}")
message(STATUS "Irrlicht library file: ${IRRLICHT_LIBRARY_FILE}")
message(STATUS "Found ${CMAKE_MATCH_COUNT} source files")

# --- Post-Build Step (Copy the DLL on Windows) ---
if(WIN32)
    add_custom_command(TARGET IrrRecastDetour POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${IRRLICHT_DLL_FILE}"
        $<TARGET_FILE_DIR:IrrRecastDetour>
        COMMENT "Copying Irrlicht.dll to executable directory"
    )
    message(STATUS "Irrlicht DLL: ${IRRLICHT_DLL_FILE}")
endif()

# --- Post-Build Step (Copy runtime assets) ---
set(IRRLICHT_MEDIA_DIR "${IRRLICHT_ROOT}/media")
if(EXISTS "${IRRLICHT_MEDIA_DIR}")
    add_custom_command(TARGET IrrRecastDetour POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different
        "${IRRLICHT_MEDIA_DIR}"
        "$<TARGET_FILE_DIR:IrrRecastDetour>/media"
        COMMENT "Copying Irrlicht 'media' directory to executable directory"
    )
    message(STATUS "Will copy media assets from: ${IRRLICHT_MEDIA_DIR}")
else()
    message(WARNING "Irrlicht media directory not found at: ${IRRLICHT_MEDIA_DIR}")
endif()