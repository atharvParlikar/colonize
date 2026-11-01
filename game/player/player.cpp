#include "player/player.hpp"

#include "collision/collision.hpp"
#include "player/hover.hpp"
#include "player/jet.hpp"
#include "world/terrain.hpp"

#include <algorithm>
#include <cmath>

using namespace engine;

namespace {

bool anyKey(const PlayerInput &input) {
  return input.forward || input.back || input.left || input.right || input.ascend || input.descend;
}

// F arms (or disarms) the next conversion; the speed gate starts it.
void stepConversion(Player &player, const PlayerInput &input, float dt) {
  if (input.convert && player.mode != FlightMode::ToJet && player.mode != FlightMode::ToHover) {
    player.convertArmed = !player.convertArmed;
  }
  switch (player.mode) {
  case FlightMode::Landed:
  case FlightMode::Hover: {
    const float forwardSpeed = Vector3DotProduct(player.velocity, directionFrom(player.lookHeading, 0.0f));
    if (player.convertArmed && forwardSpeed >= tuning.convertSpeed) {
      enterJet(player);
      player.mode = FlightMode::ToJet;
      player.convertArmed = false;
    }
    break;
  }
  case FlightMode::Jet:
    if (player.convertArmed && Vector3Length(player.velocity) <= tuning.convertBackSpeed) {
      enterHover(player);
      player.mode = FlightMode::ToHover;
      player.convertArmed = false;
    }
    break;
  case FlightMode::ToJet:
    player.convert = std::min(1.0f, player.convert + dt / tuning.convertTime);
    if (player.convert >= 1.0f) player.mode = FlightMode::Jet;
    break;
  case FlightMode::ToHover:
    player.convert = std::max(0.0f, player.convert - dt / tuning.convertTime);
    if (player.convert <= 0.0f) player.mode = FlightMode::Hover;
    break;
  }
}

void stepEngine(Player &player, const PlayerInput &input, float dt) {
  const bool resting = player.mode == FlightMode::Landed && !anyKey(input);
  player.engineIdleTime = resting ? player.engineIdleTime + dt : 0.0f;
  if (player.engineIdleTime >= kIdleTimeout) {
    player.engineRpm = 0.0f;
    return;
  }
  const float hoverLoad = std::clamp(Vector3Length(player.velocity) / tuning.hoverMaxSpeed, 0.0f, 1.0f);
  const float hoverRpm = kIdleRpm + (kTurboStartRpm - kIdleRpm) * hoverLoad;
  const float jetRpm = kIdleRpm + (kRedlineRpm - kIdleRpm) * std::pow(player.throttle, 1.2f);
  player.engineRpm = hoverRpm + (jetRpm - hoverRpm) * smoothstep(0.0f, 1.0f, player.convert);
}

} // namespace

bool isJetMode(FlightMode mode) { return mode == FlightMode::ToJet || mode == FlightMode::Jet; }

float playerAltitudeAboveGround(const Player &player) {
  return player.position.y - kEyeHeight - terrainHeight(player.position.x, player.position.z);
}

void stepPlayer(Player &player, const PlayerInput &input, float dt) {
  player.crashTimer = std::max(0.0f, player.crashTimer - dt);
  const Vector3 velocityBefore = player.velocity;

  stepConversion(player, input, dt);
  const float handOver = smoothstep(0.0f, 1.0f, player.convert);
  if (isJetMode(player.mode)) {
    stepJet(player, input, handOver, dt);
  } else {
    stepHover(player, input, 1.0f - handOver, dt);
  }

  player.position = Vector3Add(player.position, Vector3Scale(player.velocity, dt));
  const float impact = resolvePlayerCollisions(player);
  if (impact > kCrashSpeed) {
    player.crashTimer = kCrashMessageTime;
    player.crashImpact = impact;
  }
  const GroundContact contact = queryGround(player, 0.15f);
  if (contact.grounded && isJetMode(player.mode)) {
    // The jet cannot sit on the ground: touching it drops you back to hover.
    enterHover(player);
    player.mode = FlightMode::Hover;
    player.convert = 0.0f;
    player.convertArmed = false;
  }
  if (!isJetMode(player.mode) && player.mode != FlightMode::ToHover) {
    player.mode = contact.grounded && Vector3Length(player.velocity) < 1.0f && !anyKey(input) ? FlightMode::Landed
                                                                                              : FlightMode::Hover;
  }

  // Felt load along the canopy: acceleration plus the 1 g holding you up. A
  // collision's jolt is not something to black out from.
  if (impact <= 0.0f) {
    const Vector3 accel = Vector3Scale(Vector3Subtract(player.velocity, velocityBefore), 1.0f / dt);
    const Vector3 felt = Vector3Add(accel, Vector3{0.0f, kGravity, 0.0f});
    stepGLoad(player.load, Vector3DotProduct(felt, player.body.up) / kGravity, dt);
  } else {
    stepGLoad(player.load, player.load.g, dt);
  }

  stepEngine(player, input, dt);
}
