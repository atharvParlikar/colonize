#include "player/player.hpp"

#include "collision/collision.hpp"
#include "player/airframe.hpp"
#include "player/flight.hpp"
#include "player/hover.hpp"
#include "world/terrain.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

void applyMouseLook(Player &player, Vector2 mouseDelta) {
  player.yaw += mouseDelta.x * kMouseSensitivity;
  player.pitch -= mouseDelta.y * kMouseSensitivity;
  player.pitch = std::clamp(player.pitch, kMinPitch, kMaxPitch);
}

float playerAltitudeAboveGround(const Player &player) {
  return player.position.y - kEyeHeight -
         terrainHeight(player.position.x, player.position.z);
}

namespace {

float headingDeg(const Player &player) {
  return RAD2DEG * std::atan2(player.body.forward.z, player.body.forward.x);
}

void toggleLaw(Player &player) {
  const float speed = Vector3Length(player.velocity);
  if (player.law == ControlLaw::Hover) {
    player.law = ControlLaw::Flight;
    player.pitch = 0.0f;
    // Aim where the craft is going (or where the nose points when slow).
    player.aimDir = speed > 15.0f ? Vector3Scale(player.velocity, 1.0f / speed) : player.body.forward;
    player.pitchTrim = 0.0f;
    player.engineRpm = kIdleRpm;
    player.engineIdleTime = 0.0f;
  } else {
    player.law = ControlLaw::Hover;
    player.yaw = headingDeg(player);
    player.pitch = 0.0f;
  }
}

} // namespace

void stepPlayer(Player &player, const PlayerInput &input, float dt) {
  player.crashTimer = std::max(0.0f, player.crashTimer - dt);
  if (input.toggleFlight) toggleLaw(player);

  const Controls controls = player.law == ControlLaw::Flight ? flightControl(player, input, dt)
                                                             : hoverControl(player, input);
  stepAirframe(player, controls, dt);
  player.position = Vector3Add(player.position, Vector3Scale(player.velocity, dt));
  resolvePlayerCollisions(player); // indestructible for now: the impact speed is ignored

  const GroundContact contact = queryGround(player, 0.15f);
  if (contact.grounded && player.law == ControlLaw::Flight) {
    // Skids on the ground: a constant drag that thrust easily beats.
    const float along = std::hypot(player.velocity.x, player.velocity.z);
    const float keep = along > 1e-4f ? std::max(0.0f, along - kGroundFriction * dt) / along : 0.0f;
    player.velocity.x *= keep;
    player.velocity.z *= keep;
  }

  if (player.law == ControlLaw::Hover) { // in flight the engine model owns the rpm
    const float throttle = std::accumulate(player.jetThrottle.begin(), player.jetThrottle.end(), 0.0f) /
                           static_cast<float>(kJetCount);
    player.engineRpm = kIdleRpm + throttle * (kRedlineRpm - kIdleRpm);
  }

  if (contact.grounded && Vector3Length(player.velocity) < 1.0f) {
    player.mode = FlightMode::Landed;
  } else {
    player.mode = player.law == ControlLaw::Flight ? FlightMode::Flight : FlightMode::Hover;
  }
}
