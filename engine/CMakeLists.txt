# The engine: everything not specific to SKIFF. Rule: nothing here may include
# a game header. Only engine/include is on the include path of this target,
# so a stray #include "player/player.hpp" fails to compile.
add_library(engine STATIC
    src/app.cpp
    src/input/actions.cpp
    src/math/attitude.cpp
    src/collision/capsule.cpp
    src/tuning/tuning.cpp
    src/tuning/panel.cpp
)
add_library(engine::engine ALIAS engine)

target_include_directories(engine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(engine PUBLIC cxx_std_23)
target_link_libraries(engine PUBLIC ${RAYLIB_TARGET})
fps_shooter_warnings(engine)
