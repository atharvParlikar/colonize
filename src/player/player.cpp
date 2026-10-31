#include "player/player.hpp"

#include "collision/collision.hpp"
#include "player/airframe.hpp"
#include "player/pilot.hpp"
#include "world/terrain.hpp"

#include <algorithm>
#include <cmath>

float playerAltitudeAboveGround(const Player &player) {
  return player.position.y - kEyeHeight -
         terrainHeight(player.position.x, player.position.z);
}

void stepPlayer(Player &player, const PlayerInput &input, float dt) {
  player.crashTimer = std::max(0.0f, player.crashTimer - dt);

  const AirframeCommand command = pilotControl(player, input, dt);
  stepAirframe(player, command, dt);
  player.position = Vector3Add(player.position, Vector3Scale(player.velocity, dt));
  resolvePlayerCollisions(player); // indestructible for now: the impact speed is ignored

  const GroundContact contact = queryGround(player, 0.15f);
  if (contact.grounded) {
    // Skids drag along the ground with whatever weight the engine is not carrying.
    const float load = std::clamp(1.0f - player.throttle * kEngineMaxThrust / kAirframeWeight, 0.0f, 1.0f);
    const float along = std::hypot(player.velocity.x, player.velocity.z);
    const float keep = along > 1e-4f ? std::max(0.0f, along - kSkidFriction * kGravity * load * dt) / along : 0.0f;
    player.velocity.x *= keep;
    player.velocity.z *= keep;
  }

  if (contact.grounded && Vector3Length(player.velocity) < 1.0f) {
    player.mode = FlightMode::Landed;
  } else {
    player.mode = player.flight >= 0.5f ? FlightMode::Flight : FlightMode::Hover;
  }
}
