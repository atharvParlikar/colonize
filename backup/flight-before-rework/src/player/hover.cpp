#include "player/hover.hpp"

#include "player/aero.hpp"

#include <algorithm>
#include <cmath>

float shapedRate(float error, float gain, float maxRate, float maxAccel) {
  const float magnitude = std::fabs(error);
  const float brakeLimited = std::sqrt(2.0f * kBrakeMargin * maxAccel * magnitude);
  return std::copysign(std::min({gain * magnitude, brakeLimited, maxRate}), error);
}

Controls hoverControl(const Player &player, const PlayerInput &input) {
  const Attitude &body = player.body;

  // Wanted velocity from the keys, relative to the heading (player.yaw).
  const float yaw = DEG2RAD * player.yaw;
  const Vector3 heading{std::cos(yaw), 0.0f, std::sin(yaw)};
  const Vector3 side = Vector3CrossProduct(heading, kWorldUp);
  Vector3 wish = Vector3Add(Vector3Scale(heading, (input.forward ? 1.0f : 0.0f) - (input.back ? 1.0f : 0.0f)),
                            Vector3Scale(side, (input.right ? 1.0f : 0.0f) - (input.left ? 1.0f : 0.0f)));
  if (Vector3Length(wish) > 1.0f) wish = Vector3Normalize(wish);
  const Vector3 wantHorizontal = Vector3Scale(wish, kHoverMaxSpeed);
  const float wantClimb = input.ascend ? kClimbSpeed : (input.descend ? -kDescendSpeed : 0.0f);

  // Acceleration to close the velocity error, then the thrust that produces it
  // on top of weight and the aerodynamic forces.
  Vector3 accel{(wantHorizontal.x - player.velocity.x) * kHoverVelGain,
                (wantClimb - player.velocity.y) * kHoverClimbGain,
                (wantHorizontal.z - player.velocity.z) * kHoverVelGain};
  accel.y = std::clamp(accel.y, -6.0f, 8.0f);
  Vector3 thrust = Vector3Subtract(
      Vector3Scale(Vector3Add(accel, Vector3{0.0f, kGravity, 0.0f}), kAirframeMass), aeroForce(player));

  // Never ask for more tilt than the limit, and never for a sideways push
  // without some lift behind it.
  thrust.y = std::max(thrust.y, 0.25f * kAirframeWeight);
  const float maxSide = thrust.y * std::tan(DEG2RAD * kHoverMaxTiltDeg);
  const float side2 = std::hypot(thrust.x, thrust.z);
  if (side2 > maxSide) {
    thrust.x *= maxSide / side2;
    thrust.z *= maxSide / side2;
  }
  const Vector3 upWanted = Vector3Normalize(thrust);
  const float collective = std::max(0.0f, Vector3DotProduct(thrust, body.up));

  // Tilt error as a rotation axis (current up toward wanted up), plus the
  // heading error about the craft's own up axis.
  Vector3 tilt = Vector3CrossProduct(body.up, upWanted);
  const float tiltError = Vector3Length(tilt);
  if (tiltError > 1e-6f) {
    tilt = Vector3Scale(tilt, shapedRate(tiltError, kHoverTiltGain, kHoverMaxTiltRate,
                                         kMaxTiltTorque / kInertiaPitch) / tiltError);
  }

  Vector3 flatWanted = Vector3Subtract(heading, Vector3Scale(body.up, Vector3DotProduct(heading, body.up)));
  flatWanted = Vector3Length(flatWanted) > 1e-4f ? Vector3Normalize(flatWanted) : body.forward;
  const float yawError = std::atan2(Vector3DotProduct(Vector3CrossProduct(body.forward, flatWanted), body.up),
                                    Vector3DotProduct(body.forward, flatWanted));
  const float yawRate = shapedRate(yawError, kHoverYawGain, kHoverMaxYawRate, kMaxYawTorque / kInertiaYaw);

  // Rate loop: torque proportional to the rate error on each body axis.
  const Vector3 wantRate = Vector3Add(tilt, Vector3Scale(body.up, yawRate));
  const Vector3 rateError = Vector3Subtract(wantRate, player.angularVelocity);
  BodyTorque torque;
  torque.roll = kInertiaRoll * kHoverRateGain * Vector3DotProduct(rateError, body.forward);
  torque.pitch = kInertiaPitch * kHoverRateGain * Vector3DotProduct(rateError, body.right);
  torque.yaw = kInertiaYaw * kHoverRateGain * Vector3DotProduct(rateError, body.up);

  torque.roll = std::clamp(torque.roll, -kMaxTiltTorque, kMaxTiltTorque);
  torque.pitch = std::clamp(torque.pitch, -kMaxTiltTorque, kMaxTiltTorque);
  torque.yaw = std::clamp(torque.yaw, -kMaxYawTorque, kMaxYawTorque);
  Controls controls;
  controls.jets = mixLiftJets(collective, torque);
  return controls;
}
