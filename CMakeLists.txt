cmake_minimum_required(VERSION 3.28)

project(fps_shooter
    VERSION 0.1.0
    LANGUAGES C CXX
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

find_package(raylib 5.5 QUIET CONFIG)

if(TARGET raylib)
    set(RAYLIB_TARGET raylib)
elseif(TARGET raylib::raylib)
    set(RAYLIB_TARGET raylib::raylib)
else()
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(RAYLIB QUIET IMPORTED_TARGET raylib)
        if(TARGET PkgConfig::RAYLIB)
            set(RAYLIB_TARGET PkgConfig::RAYLIB)
        endif()
    endif()
endif()

if(NOT RAYLIB_TARGET)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_GAMES OFF CACHE BOOL "" FORCE)

    FetchContent_Declare(raylib
        URL https://github.com/raysan5/raylib/archive/refs/tags/5.5.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP OFF
    )
    FetchContent_MakeAvailable(raylib)

    set(RAYLIB_TARGET raylib)
endif()

function(fps_shooter_warnings target)
    if(CMAKE_CXX_COMPILER_ID MATCHES "AppleClang|Clang|GNU")
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

add_subdirectory(engine)

# SKIFF's simulation: flight model, collision, the world's shape. No window or
# GPU needed, so the game and the headless flight-sim both link it.
add_library(skiff-sim STATIC
    game/core/tuning.cpp
    game/player/player.cpp
    game/player/gload.cpp
    game/player/hover.cpp
    game/player/jet.cpp
    game/collision/capsule.cpp
    game/collision/collision.cpp
    game/world/level.cpp
    game/world/terrain.cpp
)
target_include_directories(skiff-sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/game)
target_link_libraries(skiff-sim PUBLIC engine::engine)
fps_shooter_warnings(skiff-sim)

# The game: rendering, audio, camera, input, and main.
add_executable(fps-shooter
    game/main.cpp
    game/audio/audio.cpp
    game/player/input.cpp
    game/camera/camera.cpp
    game/render/render.cpp
    game/render/scenery.cpp
    game/render/grass.cpp
)
find_package(Threads REQUIRED)
target_link_libraries(fps-shooter PRIVATE skiff-sim Threads::Threads)
if(APPLE)
    target_link_libraries(fps-shooter PRIVATE "-framework OpenGL")
endif()
fps_shooter_warnings(fps-shooter)

# Headless flight-model checks: no window, just the physics.
add_executable(flight-sim tools/flight_sim.cpp)
target_link_libraries(flight-sim PRIVATE skiff-sim)
