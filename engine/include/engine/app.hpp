#pragma once

namespace engine {

struct AppConfig {
  const char *title = "game";
  int width = 1280;
  int height = 720;
  int targetFps = 60;
  // Simulation runs on a fixed step so physics behaves the same at any frame
  // rate; rendering interpolates between the last two steps.
  float fixedDt = 1.0f / 120.0f;
  // A long frame (a hitch, a breakpoint) is clamped so the sim does not try to
  // catch up with hundreds of steps at once.
  float maxFrameTime = 0.1f;
};

// What a game provides to the engine loop. Each frame, in order:
//   input()            once: read the devices
//   fixedUpdate(dt)    0..n times, dt is always AppConfig::fixedDt
//   update(dt, alpha)  once: per-frame work (camera, audio); dt is the frame time
//   render(alpha)      once, between BeginDrawing and EndDrawing
// alpha in [0, 1) is how far real time has moved past the last fixed step,
// for interpolating what is drawn.
class Game {
public:
  virtual ~Game() = default;
  virtual void init() {} // the window exists
  virtual void shutdown() {}
  virtual void input() {}
  virtual void fixedUpdate(float dt) = 0;
  virtual void update(float dt, float alpha) {
    (void)dt;
    (void)alpha;
  }
  virtual void render(float alpha) = 0;
};

// Opens the window, runs the loop until the window closes, cleans up.
int run(const AppConfig &config, Game &game);

} // namespace engine
