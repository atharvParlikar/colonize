#include "audio/audio.hpp"
#include "camera/camera.hpp"
#include "core/controls.hpp"
#include "core/constants.hpp"
#include "core/tuning.hpp"
#include "engine/app.hpp"
#include "engine/tuning/panel.hpp"
#include "player/input.hpp"
#include "player/player.hpp"
#include "render/grass.hpp"
#include "render/render.hpp"
#include "render/scenery.hpp"
#include "raylib.h"

namespace {

class Skiff final : public engine::Game {
public:
  void init() override {
    DisableCursor();
    bindDefaultControls(controls_);
    initAudio();
    initScenery();
    initGrass();
    engine::loadTuning(kTuningFile, kFlightTuningParams); // the last saved tuning, if any

    camera_ = makeCamera(computeView(previous_, player_, 1.0f));
    preloadScenery(camera_);
  }

  void shutdown() override {
    shutdownGrass();
    shutdownScenery();
    shutdownAudio();
  }

  void input() override {
    // Tab opens the tuning panel; while it is open the mouse is the panel's
    // (the frame it toggles, the cursor jump is thrown away).
    controls_.update();
    const Vector2 mouseDelta = GetMouseDelta();
    if (controls_.pressed(Action::TuningPanel)) {
      tuningPanel_.toggle();
    } else if (!tuningPanel_.isOpen()) {
      // Mouse movement and F presses are held until a physics step consumes them.
      input_.mouseDelta = Vector2Add(input_.mouseDelta, mouseDelta);
    }
    samplePlayerInput(controls_, input_);
  }

  void fixedUpdate(float dt) override {
    previous_ = player_;
    stepPlayer(player_, input_, dt);
    input_.mouseDelta = {};
    input_.convert = false;
  }

  void update(float /*dt*/, float alpha) override {
    updateAudio(player_);
    syncCamera(camera_, computeView(previous_, player_, alpha));
  }

  void render(float /*alpha*/) override {
    renderWorld(camera_, player_);
    drawHud(player_, camera_);

    // Live readout in the tuning panel, to watch a change take effect.
    const GLoad &g = player_.load;
    tuningPanel_.draw(TextFormat("%5.0f km/h   %+5.1f g   dose %4.1f   grey %3.0f%%  black %3.0f%%   thr %3.0f%%",
                                 Vector3Length(player_.velocity) * 3.6f, g.g, g.dose, g.greyout * 100.0f,
                                 g.blackout * 100.0f, player_.throttle * 100.0f));
  }

private:
  Player player_{};
  Player previous_ = player_;
  Controls controls_{};
  PlayerInput input_{};
  Camera3D camera_{};
  engine::TuningPanel tuningPanel_{{
      .title = "FLIGHT TUNING",
      .file = kTuningFile,
      .fileHeader = "SKIFF flight tuning (written by the in-game panel)",
      .params = kFlightTuningParams,
  }};
};

} // namespace

int main() {
  Skiff game;
  return engine::run({.title = "SKIFF", .width = kScreenWidth, .height = kScreenHeight, .targetFps = 60,
                      .fixedDt = kPhysicsDt},
                     game);
}
