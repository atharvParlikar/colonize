#include "engine/app.hpp"

#include "raylib.h"

#include <algorithm>

namespace engine {

int run(const AppConfig &config, Game &game) {
  InitWindow(config.width, config.height, config.title);
  SetTargetFPS(config.targetFps);
  if (config.audio) InitAudioDevice();
  game.init();

  float accumulator = 0.0f;
  while (!WindowShouldClose()) {
    game.input();

    const float frameDt = std::min(GetFrameTime(), config.maxFrameTime);
    accumulator += frameDt;
    while (accumulator >= config.fixedDt) {
      game.fixedUpdate(config.fixedDt);
      accumulator -= config.fixedDt;
    }
    const float alpha = accumulator / config.fixedDt;

    game.update(frameDt, alpha);

    BeginDrawing();
    game.render(alpha);
    EndDrawing();
  }

  game.shutdown();
  if (config.audio && IsAudioDeviceReady()) CloseAudioDevice();
  CloseWindow();
  return 0;
}

} // namespace engine
