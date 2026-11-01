#include "audio/audio.hpp"
#include "camera/camera.hpp"
#include "collision/collision.hpp"
#include "core/constants.hpp"
#include "player/input.hpp"
#include "player/player.hpp"
#include "render/grass.hpp"
#include "render/render.hpp"
#include "render/scenery.hpp"
#include "render/tuning_ui.hpp"
#include "raylib.h"
#include <algorithm>

int main() {
  InitWindow(kScreenWidth, kScreenHeight, "SKIFF");
  SetTargetFPS(60);
  DisableCursor();
  initAudio();
  initScenery();
  initGrass();
  loadTuning(kTuningFile); // the last saved tuning, if any

  Player player{};
  Player previous = player;
  PlayerInput input{};
  Camera3D camera = makeCamera(computeView(previous, player, 1.0f));
  preloadScenery(camera);

  float accumulator = 0.0f;
  while (!WindowShouldClose()) {
    // Tab opens the tuning panel; while it is open the mouse is the panel's
    // (the frame it toggles, the cursor jump is thrown away).
    const Vector2 mouseDelta = GetMouseDelta();
    if (IsKeyPressed(KEY_TAB)) {
      toggleTuningPanel();
    } else if (!tuningPanelOpen()) {
      // Mouse movement and F presses are held until a physics step consumes them.
      input.mouseDelta = Vector2Add(input.mouseDelta, mouseDelta);
    }
    samplePlayerInput(input);

    accumulator += std::min(GetFrameTime(), 0.1f);
    while (accumulator >= kPhysicsDt) {
      previous = player;
      stepPlayer(player, input, kPhysicsDt);
      input.mouseDelta = {};
      input.convert = false;
      accumulator -= kPhysicsDt;
    }

    updateAudio(player);
    syncCamera(camera, computeView(previous, player, accumulator / kPhysicsDt));

    BeginDrawing();
    renderWorld(camera, player);
    drawHud(player, camera);
    drawTuningPanel(player);
    EndDrawing();
  }

  shutdownGrass();
  shutdownScenery();
  shutdownAudio();
  CloseWindow();
  return 0;
}
