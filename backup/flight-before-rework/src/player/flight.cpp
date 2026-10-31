#include "player/flight.hpp"

#include "player/hover.hpp"

#include <algorithm>
#include <cmath>

namespace {

float smoothstep(float a, float b, float x) {
  const float t = std::clamp((x - a) / (b - a), 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

float axisTorque(float inertia, float wantRate, float rate) {
  return inertia * kHoverRateGain * (wantRate - rate);
}

float angleBetween(const Vector3 &a, const Vector3 &b) {
  return std::atan2(Vector3Length(Vector3CrossProduct(a, b)), Vector3DotProduct(a, b));
}

// Spools the engine: W climbs to the turbo threshold at a steady rate then
// eases toward the redline, S holds just under the turbo, anything else lets
// it fall back to idle, and it shuts off after kIdleTimeout with no input.
// Returns the main-jet thrust multiple.
float stepEngine(Player &player, const PlayerInput &input, float dt) {
  float &rpm = player.engineRpm;
  const bool lit = input.forward || input.back || input.ascend || input.descend;
  if (lit) {
    player.engineIdleTime = 0.0f;
    rpm = std::max(rpm, kIdleRpm);
  }

  if (input.forward) {
    if (rpm < kTurboStartRpm) {
      rpm = std::min(rpm + kSpoolUpRate * dt, kTurboStartRpm);
    } else {
      rpm += (kRedlineRpm - rpm) / kSpoolEaseTime * dt;
    }
  } else if (input.back) {
    const float step = (rpm < kLowThrottleRpm ? kSpoolUpRate : kSpoolDownRate) * dt;
    rpm += std::clamp(kLowThrottleRpm - rpm, -step, step);
  } else if (rpm > 0.0f) {
    rpm = std::max(rpm - kSpoolDownRate * dt, kIdleRpm);
    player.engineIdleTime += dt;
    if (player.engineIdleTime > kIdleTimeout) {
      rpm = 0.0f;
    }
  }

  return input.back && !input.forward ? -kBrakeReverse : engineThrust(rpm);
}

// The mouse moves the aim point relative to the horizon, not to the cockpit:
// mouse right turns it about the world's vertical, so holding it right while
// banked flies a level circle instead of sweeping the point down the sky.
// Mouse up pitches about the craft's wing axis (levelled while it is
// near-horizontal), which stays continuous through the top of a loop. The point stays put in the world
// while the craft chases it, and may lead the nose by up to kAimMaxAngleDeg,
// so a held mouse becomes a continuous turn or loop.
void steerAim(Player &player, const PlayerInput &input) {
  const Attitude &body = player.body;

  Vector3 levelRight = Vector3Subtract(body.right, Vector3Scale(kWorldUp, Vector3DotProduct(body.right, kWorldUp)));
  const float levelLength = Vector3Length(levelRight);
  const Vector3 pitchAxis =
      levelLength > 1e-3f
          ? Vector3Normalize(Vector3Lerp(body.right, Vector3Scale(levelRight, 1.0f / levelLength),
                                         smoothstep(0.25f, 0.7f, levelLength)))
          : body.right;

  Vector3 aim = Vector3RotateByAxisAngle(player.aimDir, kWorldUp, -input.mouseDelta.x * kAimSensitivity);
  aim = Vector3RotateByAxisAngle(aim, pitchAxis, -input.mouseDelta.y * kAimSensitivity);
  aim = Vector3Normalize(aim);

  const float limit = kAimMaxAngleDeg * DEG2RAD;
  if (angleBetween(body.forward, aim) > limit) {
    Vector3 axis = Vector3CrossProduct(body.forward, aim);
    axis = Vector3Length(axis) > 1e-5f ? Vector3Normalize(axis) : body.up;
    aim = Vector3RotateByAxisAngle(body.forward, axis, limit);
  }
  player.aimDir = aim;
}

} // namespace

float engineThrust(float rpm) {
  if (rpm <= 0.0f) return 0.0f;
  if (rpm < kTurboStartRpm) return rpm / kTurboStartRpm;
  if (rpm < kTurboKickRpm) return 1.0f + (rpm - kTurboStartRpm) / (kTurboKickRpm - kTurboStartRpm);
  return std::min(2.0f + (kTurboMax - 2.0f) * (rpm - kTurboKickRpm) / (kRedlineRpm - kTurboKickRpm),
                  kTurboMax);
}

Controls flightControl(Player &player, const PlayerInput &input, float dt) {
  steerAim(player, input);
  const float thrust = stepEngine(player, input, dt);

  const Attitude &body = player.body;
  const float speed = Vector3Length(player.velocity);
  const float flying = smoothstep(8.0f, 16.0f, speed);                     // flow-referenced control fades in
  const float wingShare = smoothstep(kLiftFadeStart, kLiftFadeEnd, speed); // how much of the weight the wing carries
  const float q = dynamicPressure(player);
  const FlowAngles flow = flowAngles(player);

  // ---- Where the nose should point -----------------------------------------
  // The circle is where the pilot wants to go. Holding the nose exactly on it
  // would let the flight path sag below it by the angle of attack, so near the
  // circle the nose is aimed a little above it: proportionally, and with an
  // integrator that settles once the path lands on the circle.
  const Vector3 aim = player.aimDir;
  const Vector3 pathDir = speed > 1.0f ? Vector3Scale(player.velocity, 1.0f / speed) : body.forward;
  const float aimAngle = angleBetween(body.forward, aim);
  const float settle = flying * (1.0f - smoothstep(8.0f * DEG2RAD, 25.0f * DEG2RAD, aimAngle));
  const float above = Vector3DotProduct(Vector3Subtract(aim, pathDir), body.up);
  const float maxTrim = kMaxTrimDeg * DEG2RAD;
  player.pitchTrim += kTrimGain * above * settle * dt;
  player.pitchTrim -= player.pitchTrim * (1.0f - settle) * std::min(1.0f, 2.0f * dt); // forget it in a hard turn
  player.pitchTrim = std::clamp(player.pitchTrim, -maxTrim, maxTrim);
  // In a hard turn the flight path sinks below the circle's elevation, which
  // the trim above (close range only) cannot fix; aim higher by the sag so a
  // banked turn at the horizon holds its height. Meaningless in a loop, where
  // the circle is overhead, so it fades out toward the vertical.
  const float horizonal = 1.0f - smoothstep(0.5f, 0.85f, std::fabs(aim.y));
  const float sag = flying * horizonal * std::clamp(kSagGain * (aim.y - pathDir.y), 0.0f, 0.3f);
  const Vector3 target = Vector3Normalize(
      Vector3Add(Vector3Add(aim, Vector3Scale(body.up, player.pitchTrim + kTrimLead * above * settle)),
                 Vector3Scale(kWorldUp, sag)));

  // ---- Error to the target, as a rotation vector ---------------------------
  const Vector3 axis = Vector3CrossProduct(body.forward, target);
  const float sinErr = Vector3Length(axis);
  const float err = angleBetween(body.forward, target);
  Vector3 errorVec = sinErr > 1e-5f ? Vector3Scale(axis, err / sinErr) : Vector3Zero();
  if (sinErr <= 1e-5f && err > 1.0f) {
    errorVec = Vector3Scale(body.right, err); // dead astern: any axis will do
  }
  const float pitchError = Vector3DotProduct(errorVec, body.right);
  const float yawError = Vector3DotProduct(errorVec, body.up);

  // How far the lift side must roll to point at the target. Far from it the
  // loop rolls onto it and pulls; close to it the roll is a wings-level hold.
  const float bank = std::atan2(-body.right.y, body.up.y); // positive = right wing low
  const float targetRight = Vector3DotProduct(target, body.right);
  const float targetUp = Vector3DotProduct(target, body.up);
  const float rollToLift = std::atan2(targetRight, targetUp);
  // A target below the wing plane is reached by pushing, not by rolling
  // inverted and pulling: roll the belly toward it instead (at most 90 degrees
  // of roll either way) and command a negative pitch. The two choices blend
  // across the sideways band, where the pull is zero anyway.
  const float rollToBelly = std::atan2(-targetRight, -targetUp);
  const float push = smoothstep(80.0f * DEG2RAD, 100.0f * DEG2RAD, std::fabs(rollToLift));
  const float rollToTarget = rollToLift + push * (rollToBelly - rollToLift);
  const float pullSign = 1.0f - 2.0f * push;
  const float far = smoothstep(3.0f * DEG2RAD, 16.0f * DEG2RAD, err);


  const float horizontal = std::hypot(body.forward.x, body.forward.z);
  const float level = smoothstep(0.25f, 0.6f, horizontal); // "wings level" means nothing pointing straight up or down
  const float levelRate =
      level * shapedRate(-bank, 2.5f, 0.6f * kFlightMaxRollRate, kFlightRollAccel);
  const float chaseRate = shapedRate(rollToTarget, kFlightRollGain, kFlightMaxRollRate, kFlightRollAccel);
  float wantRoll = levelRate + far * (chaseRate - levelRate);
  if (input.left || input.right) { // A / D: roll by hand
    wantRoll = ((input.right ? 1.0f : 0.0f) - (input.left ? 1.0f : 0.0f)) * kFlightMaxRollRate;
  }

  // Pull only as much as the lift side points at the target; off to the side,
  // the yaw and the roll do the work.
  const float pullError =
      pitchError + far * (pullSign * err * std::max(std::cos(rollToTarget), 0.0f) - pitchError);
  float wantPitch = shapedRate(pullError, kFlightAttGain, kFlightMaxPitchRate, kFlightPitchAccel);

  // Fade the pull out before the wing stalls (or goes too far negative), so a
  // full-stick turn rides the edge of the lift limit instead of falling off it.
  const float margin = kAlphaMargin * DEG2RAD;
  const float headroom = wantPitch > 0.0f ? (kMaxAlphaDeg * DEG2RAD - flow.alpha) / margin
                                          : (flow.alpha - kMinAlphaDeg * DEG2RAD) / margin;
  wantPitch *= 1.0f - flying * (1.0f - std::clamp(headroom, 0.0f, 1.0f));

  // Yaw is the pitch of a banked turn as far as the horizon is concerned: it
  // holds the nose's elevation on the target. The sideslip term keeps turns
  // coordinated.
  const float wantYaw =
      shapedRate(yawError, kFlightAttGain, kFlightMaxYawRate, kFlightYawAccel) -
      kFlightBetaGain * flow.beta * flying;

  const Vector3 w = player.angularVelocity;
  BodyTorque demand;
  demand.roll = axisTorque(kInertiaRoll, wantRoll, Vector3DotProduct(w, body.forward));
  demand.pitch = axisTorque(kInertiaPitch, wantPitch, Vector3DotProduct(w, body.right));
  demand.yaw = axisTorque(kInertiaYaw, wantYaw, Vector3DotProduct(w, body.up));

  // Surfaces first, the lift jets cover what is left.
  Controls controls;
  auto split = [&](float wanted, float authority, float jetLimit, float &surface) {
    surface = std::clamp(wanted, -authority, authority);
    return std::clamp(wanted - surface, -jetLimit, jetLimit);
  };
  BodyTorque rest;
  rest.roll = split(demand.roll, q * kSurfaceRoll, kMaxTiltTorque, controls.surfaces.roll);
  rest.pitch = split(demand.pitch, q * kSurfacePitch, kMaxTiltTorque, controls.surfaces.pitch);
  rest.yaw = split(demand.yaw, q * kSurfaceYaw, kMaxYawTorque, controls.surfaces.yaw);

  // Slow, the lift jets also hold altitude (or climb on Space / Shift).
  const float wantClimb = input.ascend ? kClimbSpeed : input.descend ? -kDescendSpeed : 0.0f;
  const float liftAccel = std::clamp((wantClimb - player.velocity.y) * kHoverClimbGain, -6.0f, 8.0f);
  const float collective = (1.0f - wingShare) * kAirframeMass * (kGravity + liftAccel) /
                           std::max(body.up.y, 0.5f);
  controls.jets = mixLiftJets(collective, rest);
  controls.jets[MainLeft] = controls.jets[MainRight] = thrust;
  return controls;
}
