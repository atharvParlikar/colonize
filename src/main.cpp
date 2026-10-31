#include "audio/audio.hpp"
#include "camera/camera.hpp"
#include "collision/collision.hpp"
#include "core/constants.hpp"
#include "player/input.hpp"
#include "player/player.hpp"
#include "render/grass.hpp"
#include "render/render.hpp"
#include "render/scenery.hpp"
#include "raylib.h"
#include <algorithm>

int main() {
  InitWindow(kScreenWidth, kScreenHeight, "SKIFF");
  SetTargetFPS(60);
  DisableCursor();
  initAudio();
  initScenery();
  initGrass();

  Player player{};
  Player previous = player;
  PlayerInput input{};
  Camera3D camera = makeCamera(computeView(previous, player, 1.0f));
  preloadScenery(camera);

  float accumulator = 0.0f;
  while (!WindowShouldClose()) {
    // The mouse moves the aim point; the next physics step consumes it.
    input.mouseDelta = Vector2Add(input.mouseDelta, GetMouseDelta());
    samplePlayerInput(input);

    accumulator += std::min(GetFrameTime(), 0.1f);
    while (accumulator >= kPhysicsDt) {
      previous = player;
      stepPlayer(player, input, kPhysicsDt);
      input.mouseDelta = {};
      accumulator -= kPhysicsDt;
    }

    updateAudio(player);
    syncCamera(camera, computeView(previous, player, accumulator / kPhysicsDt));

    BeginDrawing();
    renderWorld(camera, player);
    drawHud(player, camera);
    EndDrawing();
  }

  shutdownGrass();
  shutdownScenery();
  shutdownAudio();
  CloseWindow();
  return 0;
}
