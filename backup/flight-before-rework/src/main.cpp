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
    // Hover look is applied every frame for responsiveness; the craft turns toward it.
    const Vector2 mouse = GetMouseDelta();
    if (player.law == ControlLaw::Flight) {
      input.mouseDelta = Vector2Add(input.mouseDelta, mouse); // deflects the stick in the physics step
    } else {
      applyMouseLook(player, mouse);
    }
    samplePlayerInput(input);

    accumulator += std::min(GetFrameTime(), 0.1f);
    while (accumulator >= kPhysicsDt) {
      previous = player;
      stepPlayer(player, input, kPhysicsDt);
      input.toggleFlight = false;
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
