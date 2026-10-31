#include "player/pilot.hpp"

#include "player/aero.hpp"

#include <algorithm>
#include <cmath>

namespace {

float smoothstep(float a, float b, float x) {
  const float t = std::clamp((x - a) / (b - a), 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

float angleBetween(Vector3 a, Vector3 b) {
  return std::atan2(Vector3Length(Vector3CrossProduct(a, b)), Vector3DotProduct(a, b));
}

// v without its component along the unit vector n.
Vector3 reject(Vector3 v, Vector3 n) {
  return Vector3Subtract(v, Vector3Scale(n, Vector3DotProduct(v, n)));
}

Vector3 normalizeOr(Vector3 v, Vector3 fallback) {
  const float length = Vector3Length(v);
  return length > 1e-4f ? Vector3Scale(v, 1.0f / length) : fallback;
}

Vector3 flatten(Vector3 v, Vector3 fallback) {
  return normalizeOr(Vector3{v.x, 0.0f, v.z}, fallback);
}

float axis(bool positive, bool negative) {
  return (positive ? 1.0f : 0.0f) - (negative ? 1.0f : 0.0f);
}

bool anyKey(const PlayerInput &input) {
  return input.forward || input.back || input.left || input.right || input.ascend || input.descend;
}

// Body frame as a rotation. The basis right, up, forward is left-handed
// (right = forward x up), so the matrix uses back = -forward as its z column.
Quaternion attitudeQuat(const Attitude &a) {
  const float m00 = a.right.x, m01 = a.up.x, m02 = -a.forward.x;
  const float m10 = a.right.y, m11 = a.up.y, m12 = -a.forward.y;
  const float m20 = a.right.z, m21 = a.up.z, m22 = -a.forward.z;
  Quaternion q;
  const float trace = m00 + m11 + m22;
  if (trace > 0.0f) {
    const float s = 2.0f * std::sqrt(trace + 1.0f);
    q = {(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25f * s};
  } else if (m00 > m11 && m00 > m22) {
    const float s = 2.0f * std::sqrt(1.0f + m00 - m11 - m22);
    q = {0.25f * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s};
  } else if (m11 > m22) {
    const float s = 2.0f * std::sqrt(1.0f + m11 - m00 - m22);
    q = {(m01 + m10) / s, 0.25f * s, (m12 + m21) / s, (m02 - m20) / s};
  } else {
    const float s = 2.0f * std::sqrt(1.0f + m22 - m00 - m11);
    q = {(m02 + m20) / s, (m12 + m21) / s, 0.25f * s, (m10 - m01) / s};
  }
  return QuaternionNormalize(q);
}

// Rotation (world rotation vector, rad) that takes the body onto the target.
Vector3 attitudeError(const Attitude &from, const Attitude &to) {
  Quaternion e = QuaternionMultiply(attitudeQuat(to), QuaternionInvert(attitudeQuat(from)));
  if (e.w < 0.0f) e = {-e.x, -e.y, -e.z, -e.w}; // the short way round
  const Vector3 v{e.x, e.y, e.z};
  const float s = Vector3Length(v);
  if (s < 1e-6f) return Vector3Zero();
  return Vector3Scale(v, 2.0f * std::atan2(s, e.w) / s);
}

// The mouse moves the aim point relative to the horizon: mouse X turns it
// about the world's vertical (so a held mouse flies a level circle), mouse Y
// pitches it about its own level right axis. Hovering, it stops short of
// straight up or down, like a free look. Flying, it is a spring-loaded stick,
// not a target that sticks: it may lead the flight path by at most
// kAimMaxAngleDeg, and it always eases back onto the path (kAimReturnTau), so
// a flick turns the craft part of the way and a sustained turn, loop or
// reversal takes sustained mouse movement. The pitch axis follows the wing
// near the vertical so a loop is continuous over the top.
void steerAim(Player &player, const PlayerInput &input, float dt) {
  const Attitude &body = player.body;
  const bool flying = player.flight >= 0.5f;

  // Flying, each turn of the point goes only as far as the lead limit; a turn
  // back toward the path always goes. (Clamping the result back onto the path
  // instead would drag the point's elevation along with the path's.)
  const Vector3 path = normalizeOr(player.velocity, body.forward);
  const float limit = kAimMaxAngleDeg * DEG2RAD;
  auto turnAim = [&](Vector3 from, Vector3 around, float angle) {
    const Vector3 to = Vector3RotateByAxisAngle(from, around, angle);
    const float after = angleBetween(path, to);
    if (!flying || after <= limit || after <= angleBetween(path, from)) return to;
    float lo = 0.0f, hi = 1.0f;
    for (int i = 0; i < 12; ++i) {
      const float mid = 0.5f * (lo + hi);
      (angleBetween(path, Vector3RotateByAxisAngle(from, around, angle * mid)) > limit ? hi : lo) = mid;
    }
    return Vector3RotateByAxisAngle(from, around, angle * lo);
  };

  Vector3 aim = turnAim(player.aimDir, kWorldUp, -input.mouseDelta.x * kAimSensitivity);
  Vector3 levelRight = Vector3CrossProduct(aim, kWorldUp);
  const float levelLength = Vector3Length(levelRight);
  Vector3 pitchAxis = body.right;
  if (levelLength > 1e-3f) {
    levelRight = Vector3Scale(levelRight, 1.0f / levelLength);
    if (flying) {
      if (Vector3DotProduct(levelRight, body.right) < 0.0f) levelRight = Vector3Negate(levelRight); // past the top
      pitchAxis = Vector3Normalize(Vector3Lerp(body.right, levelRight, smoothstep(0.25f, 0.7f, levelLength)));
    } else {
      pitchAxis = levelRight;
    }
  }
  aim = Vector3Normalize(turnAim(aim, pitchAxis, -input.mouseDelta.y * kAimSensitivity));

  if (!flying) {
    const float limit = kAimMaxElevationDeg * DEG2RAD;
    const float elevation = std::asin(std::clamp(aim.y, -1.0f, 1.0f));
    if (std::fabs(elevation) > limit) {
      const Vector3 flat = flatten(aim, flatten(body.forward, Vector3{0.0f, 0.0f, -1.0f}));
      const float e = std::copysign(limit, elevation);
      aim = Vector3Add(Vector3Scale(flat, std::cos(e)), Vector3Scale(kWorldUp, std::sin(e)));
    }
  } else {
    // Ease back toward the path, and never lead it by more than the limit (the
    // path can also run away from the point: a stall, a collision). The last
    // kAimHoldDeg do not spring back: that is where the released stick holds
    // the path, so the path's small lag behind the point cannot walk the point
    // (and the craft) away.
    // Eased as a heading and an elevation offset against the horizon: easing
    // along the great circle would lift a point that leads a level turn (the
    // arc bulges upward) and walk the turn into a climb. Near the vertical,
    // where heading means nothing, along the great circle.
    const float hold = kAimHoldDeg * DEG2RAD;
    const float decay = std::exp(-dt / kAimReturnTau);
    auto ease = [&](float lead) { return lead > hold ? std::min(hold + (lead - hold) * decay, limit) : lead; };
    const float lead = angleBetween(path, aim);
    const Vector3 turn = normalizeOr(Vector3CrossProduct(path, aim), body.right);
    const Vector3 arcAim = Vector3RotateByAxisAngle(path, turn, ease(lead));
    const float horizon = 1.0f - smoothstep(0.6f, 0.85f, std::max(std::fabs(path.y), std::fabs(aim.y)));
    Vector3 levelAim = arcAim;
    if (horizon > 0.0f) {
      auto headingOf = [](Vector3 v) { return std::atan2(v.x, -v.z); };
      auto elevationOf = [](Vector3 v) { return std::asin(std::clamp(v.y, -1.0f, 1.0f)); };
      const float pathHeading = headingOf(path), pathElevation = elevationOf(path);
      const float dHeading = std::remainder(headingOf(aim) - pathHeading, 2.0f * PI);
      const float dElevation = elevationOf(aim) - pathElevation;
      // Each offset eases on its own. The elevation one keeps a wider hold, so
      // the few degrees the path wanders up or down in a hard turn are not
      // soaked into the point (which would let the turn climb or sink away).
      auto easeAxis = [&](float offset, float axisHold) {
        const float m = std::fabs(offset);
        return m > axisHold ? std::copysign(axisHold + (m - axisHold) * decay, offset) : offset;
      };
      const float cosElevation = std::max(std::cos(pathElevation), 0.05f);
      float side = easeAxis(dHeading * cosElevation, hold);
      float vert = easeAxis(dElevation, kAimHoldElevationDeg * DEG2RAD);
      const float levelLead = std::hypot(side, vert);
      if (levelLead > limit) {
        side *= limit / levelLead;
        vert *= limit / levelLead;
      }
      const float heading = pathHeading + side / cosElevation;
      const float elevation = pathElevation + vert;
      levelAim = {std::sin(heading) * std::cos(elevation), std::sin(elevation), -std::cos(heading) * std::cos(elevation)};
    }
    aim = normalizeOr(Vector3Lerp(arcAim, levelAim, horizon), arcAim);
  }
  player.aimDir = aim;
}

// W raises the speed command, never more than kSpeedLead above the real
// speed; S brakes hard, dropping it up to kBrakeLead below (into reverse from a
// hover). Letting go holds the speed
// the craft has at that moment, or, below cruise, brings it to a stop.
void stepSpeedCommand(Player &player, const PlayerInput &input, float dt) {
  float &command = player.speedCommand;
  // Hovering, speed is along the aim (backing up is negative); flying, it is
  // the airspeed, wherever the aim point is.
  const float speed = Vector3DotProduct(player.velocity, player.aimDir) +
                      (Vector3Length(player.velocity) - Vector3DotProduct(player.velocity, player.aimDir)) *
                          player.flight;
  const float rate = kThrottleRateHover + (kThrottleRateFlight - kThrottleRateHover) * player.flight;
  const bool faster = input.forward && !input.back;
  const bool slower = input.back && !input.forward;
  if (faster) {
    command = std::min(command + rate * dt, std::min(kMaxSpeed, std::max(speed, 0.0f) + kSpeedLead));
  } else if (slower) {
    command = std::max(command - kBrakeRate * dt, std::max(-kReverseSpeed, speed - kBrakeLead));
  } else if (player.speedKeyHeld) {
    command = speed >= kCruiseLatch ? speed : 0.0f;
  } else if (command < kCruiseLatch) {
    command = 0.0f;
  }
  player.speedKeyHeld = faster || slower;
}

// Gauge and sound: on whenever the craft is flying or the pilot touches a
// key, off after kIdleTimeout at rest.
void stepEngineRpm(Player &player, bool resting, float dt) {
  player.engineIdleTime = resting ? player.engineIdleTime + dt : 0.0f;
  const bool on = player.engineIdleTime < kIdleTimeout;
  player.engineRpm = on ? kIdleRpm + (kRedlineRpm - kIdleRpm) * std::pow(player.throttle, 1.5f) : 0.0f;
}

// Which way (unit, across the path) and how far (rad) the flight path has to
// turn to reach the aim. Measured against the horizon, as a heading error and
// an elevation error, so a turn toward a level point stays level even when the
// point is behind (the shortest way round would arc up over the top). When
// the shortest way round goes near the vertical (a loop), where heading means
// nothing, it is the shortest way round.
Vector3 turnDirection(Vector3 path, Vector3 aim, float &error) {
  const Vector3 greatCircle = normalizeOr(reject(aim, path), Vector3Zero());
  const float greatError = angleBetween(path, aim);
  float steepest = std::max(std::fabs(path.y), std::fabs(aim.y));
  for (float t : {0.25f, 0.5f, 0.75f}) { // along the arc
    const Vector3 along = Vector3RotateByAxisAngle(path, normalizeOr(Vector3CrossProduct(path, aim), kWorldUp),
                                                   greatError * t);
    steepest = std::max(steepest, std::fabs(along.y));
  }
  const float horizon = 1.0f - smoothstep(0.6f, 0.85f, steepest);
  if (horizon <= 0.0f) {
    error = greatError;
    return greatCircle;
  }
  const Vector3 pathFlat = flatten(path, Vector3{0.0f, 0.0f, -1.0f});
  const Vector3 aimFlat = flatten(aim, pathFlat);
  const float heading = std::atan2(-Vector3CrossProduct(pathFlat, aimFlat).y, Vector3DotProduct(pathFlat, aimFlat));
  const float elevation = std::asin(std::clamp(aim.y, -1.0f, 1.0f)) - std::asin(std::clamp(path.y, -1.0f, 1.0f));
  const Vector3 right = normalizeOr(Vector3CrossProduct(path, kWorldUp), pathFlat);
  const Vector3 up = Vector3CrossProduct(right, path);
  const float sideways = heading * std::sqrt(std::max(0.0f, 1.0f - path.y * path.y));
  const Vector3 levelled = normalizeOr(Vector3Add(Vector3Scale(right, sideways), Vector3Scale(up, elevation)), greatCircle);
  const float levelError = std::hypot(sideways, elevation);
  error = greatError + (levelError - greatError) * horizon;
  return normalizeOr(Vector3Lerp(greatCircle, levelled, horizon), levelled);
}

} // namespace

float shapedRate(float error, float gain, float maxRate, float maxAccel) {
  const float magnitude = std::fabs(error);
  const float brakeLimited = std::sqrt(2.0f * kBrakeMargin * maxAccel * magnitude);
  return std::copysign(std::min({gain * magnitude, brakeLimited, maxRate}), error);
}

AirframeCommand pilotControl(Player &player, const PlayerInput &input, float dt) {
  player.flight = smoothstep(kWingSpeedLo, kWingSpeedHi, Vector3Length(player.velocity));
  steerAim(player, input, dt);
  stepSpeedCommand(player, input, dt);

  const Attitude &body = player.body;
  const float flight = player.flight;
  const float hover = 1.0f - flight;
  const Vector3 aim = player.aimDir;
  const float speed = Vector3Length(player.velocity);
  const Vector3 path = speed > 1.0f ? Vector3Scale(player.velocity, 1.0f / speed) : body.forward;

  // ---- Wanted acceleration ---------------------------------------------------
  // Hover: a velocity along the aim, plus strafe and climb, closed with a
  // proportional loop.
  const Vector3 heading = flatten(aim, flatten(body.forward, Vector3{0.0f, 0.0f, -1.0f}));
  const Vector3 side = Vector3CrossProduct(heading, kWorldUp);
  const float climb = input.ascend ? kClimbSpeed : (input.descend ? -kDescendSpeed : 0.0f);
  const Vector3 wantVelocity =
      Vector3Add(Vector3Scale(aim, player.speedCommand),
                 Vector3Scale(Vector3Add(Vector3Scale(side, kStrafeSpeed * axis(input.right, input.left)),
                                         Vector3Scale(kWorldUp, climb)),
                              hover));
  const Vector3 velocityError = Vector3Subtract(wantVelocity, player.velocity);
  // Slowing down may be harder than speeding up (the airbrake helps).
  Vector3 hoverAccel{velocityError.x * kHoverVelGain, 0.0f, velocityError.z * kHoverVelGain};
  const float flatAccel = std::hypot(hoverAccel.x, hoverAccel.z);
  const float hoverLimit =
      hoverAccel.x * player.velocity.x + hoverAccel.z * player.velocity.z < 0.0f ? kHoverMaxBrake : kHoverMaxAccel;
  if (flatAccel > hoverLimit) hoverAccel = Vector3Scale(hoverAccel, hoverLimit / flatAccel);
  hoverAccel.y = std::clamp(velocityError.y * kHoverClimbGain, -6.0f, 8.0f);

  // Flight: turn the flight path toward the aim as hard as the wing can pull
  // (lift at kTurnAlphaDeg, less what holds the weight up), but no faster than
  // kMaxTurnRateDeg, and hold the commanded speed along it. No g limit.
  // Lift per radian of angle of attack: the wing's, plus the hull's belly, which
  // pushes up like a flat plate once the nose is above the flow.
  const float liftSlope = std::max(dynamicPressure(player), 1.0f) * (kWingArea * kLiftSlope + kHullDragUp);
  const float pullAccel = liftSlope * kTurnAlphaDeg * DEG2RAD / kAirframeMass;
  const float pullTurn = std::sqrt(std::max(pullAccel * pullAccel - kGravity * kGravity, 0.0f));
  float pathError = 0.0f;
  const Vector3 across = turnDirection(path, aim, pathError);
  const float turnRate = shapedRate(pathError, kPathGain,
                                    std::min(kMaxTurnRateDeg * DEG2RAD, pullTurn / std::max(speed, 1.0f)), kPathAccel);
  const Vector3 flightAccel =
      Vector3Add(Vector3Scale(across, speed * turnRate),
                 Vector3Scale(path, std::clamp(kSpeedGain * (player.speedCommand - speed), -kFlightMaxBrake,
                                               kFlightMaxAccel)));

  const Vector3 accel = Vector3Lerp(hoverAccel, flightAccel, flight);
  const Vector3 force = Vector3Scale(Vector3Add(accel, Vector3{0.0f, kGravity, 0.0f}), kAirframeMass);

  // What the engine has to make on top of the wing and hull as they are now.
  Vector3 need = Vector3Subtract(force, aeroForce(player));

  // How far the nozzle may swing aft / forward: it opens up with forward speed.
  const float convert = smoothstep(kConvertStart, kConvertEnd, Vector3DotProduct(player.velocity, body.forward));

  // ---- Wanted attitude -------------------------------------------------------
  // Hover: tilt the body so the thrust lines up with what is needed (within
  // the tilt limit), face the aim. Once moving forward the nozzle makes the
  // fore/aft push instead of a pitch, which with the wing flying would just
  // add lift.
  Vector3 tilted = Vector3Subtract(need, Vector3Scale(heading, Vector3DotProduct(need, heading) * convert));
  tilted.y = std::max(tilted.y, 0.3f * kAirframeWeight);
  const float maxSide = tilted.y * std::tan(kHoverMaxTiltDeg * DEG2RAD);
  const float sideForce = std::hypot(tilted.x, tilted.z);
  if (sideForce > maxSide) {
    tilted.x *= maxSide / sideForce;
    tilted.z *= maxSide / sideForce;
  }
  const Vector3 hoverUp = Vector3Normalize(tilted);
  const Vector3 hoverForward = normalizeOr(reject(heading, hoverUp), body.forward);

  // Flight: the wing makes the part of the force across the flight path. Split
  // it, in the path's own horizon frame, into a sideways part and a vertical
  // part. Three ways to fly it:
  //   turn   - bank into the turn and pull, as a level turn would:
  //            tan(bank) = sideways / vertical (floored, below, so a near-zero
  //            vertical cannot snap the bank);
  //   push   - a mostly-down demand (a pushover): wings level, negative lift,
  //            any sideways part by skidding. Never bank away from the turn and
  //            push, which turns the right way but rolls the wrong way;
  //   invert - rolled past knife edge and pulling (the top of a loop, a split
  //            S), only when the wing is already nearer to that than the others.
  // (Banking along the force's direction, as this used to, is ill-conditioned:
  // pushing over, any sideways crumb pointed the wing at knife edge.)
  const Vector3 lift = reject(force, path);
  const Vector3 currentUp = normalizeOr(reject(body.up, path), body.up);
  // Near the vertical the horizon means nothing; measure against the canopy.
  const Vector3 upRef = normalizeOr(Vector3Lerp(kWorldUp, currentUp, smoothstep(0.8f, 0.97f, std::fabs(path.y))),
                                    currentUp);
  const Vector3 pathRight = normalizeOr(Vector3CrossProduct(path, upRef), body.right);
  const Vector3 pathUp = Vector3CrossProduct(pathRight, path);
  const float vertical = Vector3DotProduct(lift, pathUp);
  const float sidewaysAll = Vector3DotProduct(lift, pathRight);
  // Small sideways corrections are flown flat, by skidding (yawing the nose a
  // few degrees so the flanks push), and only a real turn banks: below
  // kFlatTurnDeg of sideways aim error the bank fades out (the mouse-aim
  // "wings level vs. aggressive roll" blend), so nudging the aim does not rock
  // the wings.
  const float sidewaysError = pathError * std::fabs(Vector3DotProduct(across, pathRight));
  const float banked = smoothstep(0.25f * kFlatTurnDeg * DEG2RAD, kFlatTurnDeg * DEG2RAD, sidewaysError);
  // The choices use what the wing is doing now (pulling or pushing) as a
  // hysteresis memory, so they do not chatter at their boundaries.
  const bool pulling = flowAngles(player).alpha >= 0.0f;
  const float band = 0.3f * kAirframeWeight;
  const bool upward = vertical > band || (vertical > -band && pulling);
  const bool turning = upward || banked * std::fabs(sidewaysAll) > (pulling ? 0.7f : 1.4f) * std::fabs(vertical);
  // The vertical part as the bank sees it: when it is small or points down it
  // gives no reference, so the bank is the 1 g level-turn one (tan = sideways /
  // weight); when it clearly points up, at least half a weight, so a turn can
  // still steepen to sink.
  const float floored =
      std::max(std::fabs(vertical), kAirframeWeight * (1.0f - 0.5f * smoothstep(-band, band, vertical)));
  const float turnBank = banked * std::atan2(sidewaysAll, floored);    // |bank| < 90, faded in like the blend
  const float invertBank = std::atan2(banked * sidewaysAll, -floored); // |bank| > 90
  const float bankNow = std::atan2(Vector3DotProduct(currentUp, pathRight), Vector3DotProduct(currentUp, pathUp));
  auto rollTo = [&](float b) { return std::fabs(std::remainder(b - bankNow, 2.0f * PI)); };
  const float primaryBank = turning ? turnBank : 0.0f;
  const bool inverted = !upward && rollTo(invertBank) < rollTo(primaryBank) - kPushPreferDeg * DEG2RAD;
  const bool pushing = !turning && !inverted;
  const float bank = inverted ? invertBank : primaryBank;
  // What the bank carries sideways (a coordinated bank of that angle).
  const float sideways = pushing ? 0.0f : (inverted ? banked * sidewaysAll : floored * std::tan(bank));
  const Vector3 wingUp = Vector3Add(Vector3Scale(pathUp, std::cos(bank)), Vector3Scale(pathRight, std::sin(bank)));
  // How much lift: the best fit to what is wanted for where the wing points
  // now (so the pull comes as the bank arrives: roll, then pull, instead of
  // pulling straight up while still rolling in), with the vertical part
  // weighted 2x so a shallow bank holds the climb or dive instead of ballooning.
  const float fitBank = bankNow + 0.5f * std::remainder(bank - bankNow, 2.0f * PI);
  const float c = std::cos(fitBank), s = std::sin(fitBank);
  const float liftAlong = std::max((4.0f * vertical * c + sideways * s) / (4.0f * c * c + s * s),
                                   -kMaxPushG * kAirframeWeight);
  const float alpha = std::clamp(liftAlong / liftSlope, kMinAlphaDeg * DEG2RAD, kMaxAlphaDeg * DEG2RAD);
  // The skid makes up the sideways push the wing is not making yet: all of a
  // flat correction, the rest of a turn while the bank is still rolling in.
  const float sideSlope = std::max(dynamicPressure(player), 1.0f) * kWingArea * kSideForceSlope;
  const float skid = std::clamp((sidewaysAll - liftAlong * std::sin(bankNow)) / sideSlope,
                                -kMaxSkidDeg * DEG2RAD, kMaxSkidDeg * DEG2RAD);
  const Vector3 wingRight = Vector3CrossProduct(path, wingUp);
  const Vector3 flightForward = Vector3Normalize(Vector3Add(
      Vector3Add(Vector3Scale(path, std::cos(alpha)), Vector3Scale(wingUp, std::sin(alpha))),
      Vector3Scale(wingRight, std::tan(skid)))); // the nose leads the path into a flat turn
  const Vector3 flightUp = Vector3Subtract(Vector3Scale(wingUp, std::cos(alpha)), Vector3Scale(path, std::sin(alpha)));

  const Attitude target =
      makeAttitude(normalizeOr(Vector3Lerp(hoverForward, flightForward, flight), body.forward),
                   Vector3Lerp(hoverUp, flightUp, flight));

  // ---- Rates -----------------------------------------------------------------
  const Vector3 error = attitudeError(body, target);
  float roll = shapedRate(Vector3DotProduct(error, body.forward), kAttitudeGain, kMaxRollRate, kRollAccel);
  const float pitch = shapedRate(Vector3DotProduct(error, body.right), kAttitudeGain, kMaxPitchRate, kPitchAccel);
  const float yaw = shapedRate(Vector3DotProduct(error, body.up), kAttitudeGain, kMaxYawRate, kYawAccel);
  if ((input.left || input.right) && flight > 0.0f) { // A / D at speed: roll by hand
    roll += (axis(input.right, input.left) * kManualRollRate - roll) * flight;
  }

  AirframeCommand command;
  command.rate = Vector3Add(Vector3Add(Vector3Scale(body.forward, roll), Vector3Scale(body.right, pitch)),
                            Vector3Scale(body.up, yaw));
  // In a steady turn the target itself rotates with the flight path (v x a / v^2);
  // feeding that forward means the loop needs no standing error to keep up,
  // so the nose really sits at the angle of attack the wing needs. The turn
  // is the one the planned lift makes (a push is capped), not the one asked for.
  const Vector3 turnAccel = Vector3Subtract(Vector3Add(Vector3Scale(Vector3Add(Vector3Scale(pathUp, c), Vector3Scale(pathRight, s)), liftAlong / kAirframeMass),
                                                       Vector3Scale(wingRight, sideSlope * skid / kAirframeMass)),
                                            reject(Vector3{0.0f, kGravity, 0.0f}, path));
  command.rate = Vector3Add(command.rate,
                            Vector3Scale(Vector3CrossProduct(path, turnAccel), flight / std::max(speed, 1.0f)));

  // ---- Engine ----------------------------------------------------------------
  // Whatever the wing and hull do not provide, the engine does, through the
  // nozzle. Hovering the nozzle stays near vertical (moving means tilting); as
  // forward speed builds it opens up to fully aft. Whenever the engine would
  // have to push backwards, the airbrake opens first; jet-borne the nozzle also
  // tips forward for whatever the airbrake cannot do.
  const float brakeDrag = dynamicPressure(player) * kAirbrakeArea;
  if (brakeDrag > 1.0f) {
    command.airbrake = std::clamp(-Vector3DotProduct(need, path) / brakeDrag, 0.0f, 1.0f);
    need = Vector3Add(need, Vector3Scale(path, command.airbrake * brakeDrag));
  }
  const float hoverTravel = kNozzleHoverDeg * DEG2RAD;
  const float maxNozzle = hoverTravel + (kNozzleAftDeg * DEG2RAD - hoverTravel) * convert;
  const float minNozzle = (-hoverTravel + (kNozzleBrakeDeg * DEG2RAD + hoverTravel) * convert) * hover;
  const float wantNozzle = std::clamp(std::atan2(Vector3DotProduct(need, body.forward), Vector3DotProduct(need, body.up)),
                                      minNozzle, maxNozzle);
  command.nozzle = player.nozzle + std::clamp(wantNozzle - player.nozzle, -kNozzleRate * dt, kNozzleRate * dt);
  command.thrust = std::max(0.0f, Vector3DotProduct(need, nozzleDirection(player, command.nozzle)));

  // Parked with hands off: engine to idle, the craft sits on its skids.
  const bool resting = player.mode == FlightMode::Landed && !anyKey(input);
  if (resting) command.thrust = 0.0f;
  stepEngineRpm(player, resting, dt);
  return command;
}
