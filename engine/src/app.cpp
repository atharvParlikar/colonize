#include "engine/app.hpp"

#include "raylib.h"

#include <algorithm>

namespace engine {

int run(const AppConfig &config, Game &game) {
  InitWindow(config.width, config.height, config.title);
  SetTargetFPS(config.targetFps);
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
  CloseWindow();
  return 0;
}

} // namespace engine
