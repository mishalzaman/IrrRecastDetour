# --- Define files common to all demos ---
set(DEMO_COMMON_SOURCES
    "common/InputEventReceiver.cpp"
    "common/InputEventReceiver.h"
    "common/Config.h"
)

# --- Demo 1: Static NavMesh ---
add_executable(Demo_Static
    "Demo_Static/Demo_Static.cpp"
    ${DEMO_COMMON_SOURCES}
)

# --- Demo 2: Tiled NavMesh ---
add_executable(Demo_Tiled
    "Demo_Tiled/Demo_Tiled.cpp"
    ${DEMO_COMMON_SOURCES}
)

# -- Realms
add_executable(Realms
    "Realms/Realms.cpp"
    ${DEMO_COMMON_SOURCES}
)

# --- Link Demos ---
set(DEMO_TARGETS
    Demo_Static
    Demo_Tiled
    Realms
)

foreach(DEMO_TARGET ${DEMO_TARGETS})
    # Link our navmesh library (CMake finds this target)
    target_link_libraries(${DEMO_TARGET} PRIVATE IrrRecastDetour)
    
    # Link Irrlicht (The path is inherited from the parent file)
    target_link_libraries(${DEMO_TARGET} PRIVATE ${IRRLICHT_LIBRARY_FILE})
    
    # Additional Windows-specific libraries
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
    # Note: Asset paths are relative to the *root* CMakeLists.txt
    set(ASSETS_DEST_DIR "$<TARGET_FILE_DIR:${DEMO_TARGET}>/assets")
    set(ASSETS_SOURCE_DIR "${CMAKE_SOURCE_DIR}/assets")
    
    if(EXISTS "${ASSETS_SOURCE_DIR}")
        add_custom_command(TARGET ${DEMO_TARGET} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different
            "${ASSETS_SOURCE_DIR}"
            "${ASSETS_DEST_DIR}"
            COMMENT "Copying 'assets' directory to ${DEMO_TARGET} directory"
        )
    else()
        message(WARNING "assets directory not found at: ${ASSETS_SOURCE_DIR}")
    endif()
endforeach()

# --- Organize files in Visual Studio ---
source_group("Demos" FILES 
    "Demo_Static/Demo_Static.cpp"
    "Demo_Tiled/Demo_Tiled.cpp"
    "Realms/Realms.cpp"
)
source_group("Demo Common" FILES ${DEMO_COMMON_SOURCES})
