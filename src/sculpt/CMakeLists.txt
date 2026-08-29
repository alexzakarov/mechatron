# Sculpt Module (Stage 4)
# Provides Blender-style sculpt brushes (Draw/Grab/Smooth/Pinch/Inflate/Flatten/Mask)
# implemented over EditableMesh. The seven first-class algorithms cover all 29 of
# Blender's brushes via BrushPreset parameter sets.

target_sources(mechatron_core
    PRIVATE
        SculptEngine.cpp
)

target_include_directories(mechatron_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)
