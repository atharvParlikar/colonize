#include "player/airframe.hpp"

#include "player/aero.hpp"
#include "player/player.hpp"

#include <algorithm>
#include <cmath>

Vector3 nozzleDirection(const Player &player, float nozzle) {
  return Vector3Add(Vector3Scale(player.body.up, std::cos(nozzle)),
                    Vector3Scale(player.body.forward, std::sin(nozzle)));
}

Vector3 hullDrag(const Player &player) {
  // Streamlined along the nose, a flat plate across the belly and flanks.
  const Attitude &b = player.body;
  const float rho = airDensity(player.position.y);
  auto axis = [&](Vector3 dir, float cda) {
    const float v = Vector3DotProduct(player.velocity, dir);
    return Vector3Scale(dir, -0.5f * rho * cda * std::fabs(v) * v);
  };
  return Vector3Add(Vector3Add(axis(b.forward, kHullDragForward), axis(b.right, kHullDragSide)),
                    axis(b.up, kHullDragUp));
}

Vector3 aeroForce(const Player &player) {
  return Vector3Add(hullDrag(player),
                    computeWingsuitAero(player.velocity, player.body, player.position.y).force);
}

float dynamicPressure(const Player &player) {
  const float speed = Vector3Length(player.velocity);
  return 0.5f * airDensity(player.position.y) * speed * speed;
}

FlowAngles flowAngles(const Player &player) {
  FlowAngles out;
  const float speed = Vector3Length(player.velocity);
  out.airspeed = speed;
  if (speed < 0.5f) return out;
  const Attitude &b = player.body;
  out.alpha = std::atan2(-Vector3DotProduct(player.velocity, b.up),
                         Vector3DotProduct(player.velocity, b.forward));
  out.beta = std::asin(std::clamp(Vector3DotProduct(player.velocity, b.right) / speed, -1.0f, 1.0f));
  return out;
}

void stepAirframe(Player &player, const AirframeCommand &command, float dt) {
  player.nozzle = command.nozzle;
  player.airbrake = std::clamp(command.airbrake, 0.0f, 1.0f);
  const float wanted = std::clamp(command.thrust / kEngineMaxThrust, 0.0f, 1.0f);
  player.throttle += (wanted - player.throttle) * std::min(1.0f, dt / kEngineSpoolTau);

  const Vector3 thrust = Vector3Scale(nozzleDirection(player, player.nozzle), player.throttle * kEngineMaxThrust);
  const Vector3 brake = Vector3Scale(player.velocity, -0.5f * airDensity(player.position.y) * kAirbrakeArea *
                                                         player.airbrake * Vector3Length(player.velocity));
  const Vector3 force = Vector3Add(Vector3Add(thrust, brake), aeroForce(player));
  const Vector3 accel = Vector3Add(Vector3{0.0f, -kGravity, 0.0f}, Vector3Scale(force, 1.0f / kAirframeMass));
  player.velocity = Vector3Add(player.velocity, Vector3Scale(accel, dt));

  // Fly-by-wire: the body rate follows the command with a short lag.
  player.angularVelocity = Vector3Add(
      player.angularVelocity,
      Vector3Scale(Vector3Subtract(command.rate, player.angularVelocity), std::min(1.0f, dt / kAttitudeTau)));

  const float rate = Vector3Length(player.angularVelocity);
  if (rate > 1e-6f) {
    const Vector3 axis = Vector3Scale(player.angularVelocity, 1.0f / rate);
    const float angle = rate * dt;
    const Vector3 forward = Vector3RotateByAxisAngle(player.body.forward, axis, angle);
    const Vector3 up = Vector3RotateByAxisAngle(player.body.up, axis, angle);
    player.body = makeAttitude(forward, up); // re-orthonormalise
  }
}
