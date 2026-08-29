# Paint Module (Stage 5)
# Provides Vertex/Weight/Texture paint brushes over EditableMesh. Reuses
# SculptEngine's falloff math and SpatialHash.

target_sources(mechatron_core
    PRIVATE
        PaintEngine.cpp
)

target_include_directories(mechatron_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)
