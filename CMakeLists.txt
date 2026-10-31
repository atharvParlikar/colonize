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

add_executable(fps-shooter
    src/main.cpp
    src/audio/audio.cpp
    src/player/player.cpp
    src/player/aero.cpp
    src/player/airframe.cpp
    src/player/pilot.cpp
    src/player/input.cpp
    src/camera/camera.cpp
    src/render/render.cpp
    src/render/scenery.cpp
    src/render/grass.cpp
    src/collision/capsule.cpp
    src/collision/collision.cpp
    src/world/level.cpp
    src/world/terrain.cpp
)

target_include_directories(fps-shooter PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_compile_features(fps-shooter PRIVATE cxx_std_23)
find_package(Threads REQUIRED)
target_link_libraries(fps-shooter PRIVATE ${RAYLIB_TARGET} Threads::Threads)
if(APPLE)
  target_link_libraries(fps-shooter PRIVATE "-framework OpenGL")
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "AppleClang|Clang|GNU")
    target_compile_options(fps-shooter PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Headless flight-model checks: no window, just the physics.
add_executable(flight-sim
    tools/flight_sim.cpp
    src/player/player.cpp
    src/player/aero.cpp
    src/player/airframe.cpp
    src/player/pilot.cpp
    src/collision/capsule.cpp
    src/collision/collision.cpp
    src/world/level.cpp
    src/world/terrain.cpp
)
target_include_directories(flight-sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(flight-sim PRIVATE ${RAYLIB_TARGET})
