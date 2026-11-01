#include "player/hover.hpp"

#include <algorithm>
#include <cmath>

using namespace engine;

namespace {

float axis(bool positive, bool negative) { return (positive ? 1.0f : 0.0f) - (negative ? 1.0f : 0.0f); }

// Moves v toward target by at most maxStep.
Vector3 moveToward(Vector3 v, Vector3 target, float maxStep) {
  const Vector3 d = Vector3Subtract(target, v);
  const float length = Vector3Length(d);
  return length <= maxStep || length < 1e-6f ? target : Vector3Add(v, Vector3Scale(d, maxStep / length));
}

} // namespace

void enterHover(Player &player) {
  player.lookHeading = headingOf(player.body.forward);
  player.lookPitch = std::clamp(pitchOf(player.body), -60.0f * DEG2RAD, 60.0f * DEG2RAD);
}

void stepHover(Player &player, const PlayerInput &input, float authority, float dt) {
  // Look.
  player.lookHeading = wrapAngle(player.lookHeading + input.mouseDelta.x * tuning.lookSensitivity);
  player.lookPitch = std::clamp(player.lookPitch - input.mouseDelta.y * tuning.lookSensitivity,
                                -tuning.lookMaxPitchDeg * DEG2RAD, tuning.lookMaxPitchDeg * DEG2RAD);

  // Wanted velocity, relative to the look heading.
  const Vector3 ahead = directionFrom(player.lookHeading, 0.0f);
  const Vector3 side{-ahead.z, 0.0f, ahead.x}; // right of ahead
  const float move = axis(input.forward, input.back);
  const float forwardSpeed = move > 0.0f ? tuning.hoverMaxSpeed : tuning.hoverReverseSpeed;
  const Vector3 wantFlat = Vector3Add(Vector3Scale(ahead, move * forwardSpeed),
                                      Vector3Scale(side, axis(input.right, input.left) * tuning.hoverStrafeSpeed));
  const float wantUp = input.ascend ? tuning.hoverClimbSpeed : (input.descend ? -tuning.hoverDescendSpeed : 0.0f);

  // Ramp to it: faster when slowing down than when speeding up.
  const Vector3 flat{player.velocity.x, 0.0f, player.velocity.z};
  const bool slowing = Vector3DotProduct(Vector3Subtract(wantFlat, flat), flat) < 0.0f;
  const float ramp = std::max(authority, 0.15f) * dt;
  const Vector3 newFlat = moveToward(flat, wantFlat, (slowing ? tuning.hoverBrake : tuning.hoverAccel) * ramp);
  float vy = player.velocity.y;
  vy += std::clamp(wantUp - vy, -tuning.hoverVerticalAccel * ramp, tuning.hoverVerticalAccel * ramp);
  const Vector3 accel = Vector3Scale(Vector3Subtract(Vector3{newFlat.x, vy, newFlat.z}, player.velocity), 1.0f / dt);
  player.velocity = {newFlat.x, vy, newFlat.z};

  // Cosmetic body: faces the look heading, leans into the acceleration.
  const float forwardAccel = Vector3DotProduct(accel, ahead);
  const float sideAccel = Vector3DotProduct(accel, side);
  const float maxTilt = tuning.hoverMaxTiltDeg * DEG2RAD;
  const float lean = std::clamp(-forwardAccel * tuning.hoverTiltPerAccel * DEG2RAD, -maxTilt, maxTilt);
  const float bank = std::clamp(sideAccel * tuning.hoverTiltPerAccel * DEG2RAD, -maxTilt, maxTilt);
  Attitude target = makeAttitude(directionFrom(player.lookHeading, lean), kWorldUp);
  target = makeAttitude(target.forward, Vector3RotateByAxisAngle(target.up, target.forward, bank));
  player.body = blendAttitude(player.body, target, 1.0f - std::exp(-tuning.hoverTurnRate * authority * dt));
}
