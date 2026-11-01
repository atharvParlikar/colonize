#include "player/jet.hpp"

#include <algorithm>
#include <cmath>

using namespace engine;

namespace {

float axis(bool positive, bool negative) { return (positive ? 1.0f : 0.0f) - (negative ? 1.0f : 0.0f); }

// Rotates v toward target by the fraction k of the angle between them.
Vector3 turnToward(Vector3 v, Vector3 target, float k) {
  const Vector3 cross = Vector3CrossProduct(v, target);
  const float s = Vector3Length(cross);
  if (s < 1e-6f) return v;
  const float angle = std::atan2(s, Vector3DotProduct(v, target));
  return Vector3Normalize(Vector3RotateByAxisAngle(v, Vector3Scale(cross, 1.0f / s), angle * k));
}

} // namespace

void enterJet(Player &player) {
  const float speed = Vector3Length(player.velocity);
  player.body = makeAttitude(directionFrom(player.lookHeading, std::clamp(player.lookPitch, -0.5f, 0.5f)), kWorldUp);
  player.throttle = std::clamp(jetDrag() * speed * speed / tuning.jetThrustAccel, 0.0f, 1.0f);
  player.stick = {};
  player.rollTrim = 0.0f;
}

void stepJet(Player &player, const PlayerInput &input, float authority, float dt) {
  Attitude body = player.body;
  float speed = std::max(Vector3Length(player.velocity), kJetMinSpeed);
  Vector3 path = Vector3Length(player.velocity) > 1.0f ? Vector3Normalize(player.velocity) : body.forward;
  const float blackout = player.load.blackout;

  // ---- Stick: mouse deflects it, it springs back ---------------------------
  Vector2 stick = player.stick;
  stick.x += input.mouseDelta.x * tuning.stickSensitivity;
  stick.y -= input.mouseDelta.y * tuning.stickSensitivity;
  stick = Vector2Scale(stick, std::exp(-dt / tuning.stickReturnTau));
  const float deflection = Vector2Length(stick);
  if (deflection > 1.0f) stick = Vector2Scale(stick, 1.0f / deflection);
  player.stick = stick;

  // ---- Rates ---------------------------------------------------------------
  // Full stick is tuning.jetMaxRateDeg, but never more than the g allowed at this
  // speed (less when blacked out), and soft near the stall.
  const float lift = std::clamp(speed * speed / (tuning.jetStallSpeed * tuning.jetStallSpeed), 0.0f, 1.0f);
  const float maxG = tuning.jetMaxG + (tuning.gLocMaxG - tuning.jetMaxG) * blackout;
  const float maxRate = std::min(tuning.jetMaxRateDeg * DEG2RAD, maxG * kGravity / speed) * lift;
  const float hand = authority * (1.0f - tuning.gLocStickLoss * blackout);
  float turnRate = stick.x * maxRate * hand;  // heading, right positive
  float pitchRate = stick.y * maxRate * hand; // about the wing, nose up positive
  const float cosPitch = std::cos(pitchOf(body));
  const float combined = std::hypot(turnRate * cosPitch, pitchRate);
  if (combined > maxRate) {
    turnRate *= maxRate / combined;
    pitchRate *= maxRate / combined;
  }

  // Roll: auto-bank to the coordinated bank of the turn, plus what A / D set.
  // Near the vertical the horizon means nothing, so the auto-bank lets go.
  const float bank = bankOf(body);
  const float coordinated = std::clamp(std::atan2(speed * turnRate * cosPitch, kGravity),
                                       -tuning.jetMaxAutoBankDeg * DEG2RAD, tuning.jetMaxAutoBankDeg * DEG2RAD);
  float rollRate;
  const float rollKeys = axis(input.right, input.left);
  if (rollKeys != 0.0f) {
    rollRate = rollKeys * tuning.jetRollRate;
    player.rollTrim = wrapAngle(bank - coordinated); // letting go holds this bank
  } else {
    player.rollTrim *= std::exp(-dt / tuning.rollTrimReturnTau);
    const float horizon = 1.0f - smoothstep(0.85f, 0.98f, std::fabs(body.forward.y));
    rollRate = horizon * std::clamp(tuning.jetRollGain * wrapAngle(coordinated + player.rollTrim - bank),
                                    -tuning.jetMaxAutoRollRate, tuning.jetMaxAutoRollRate);
  }

  // Heading turns about the world vertical, pitch about the wing, roll about
  // the nose.
  const Vector3 omega = Vector3Add(Vector3Add(Vector3Scale(kWorldUp, -turnRate), Vector3Scale(body.right, pitchRate)),
                                   Vector3Scale(body.forward, rollRate));
  body = rotateAttitude(body, omega, dt);

  // ---- Flight path ----------------------------------------------------------
  // Follows the nose with a lag; below the stall the wing lets it (and the
  // nose) sag under the weight it can no longer carry.
  path = turnToward(path, body.forward, 1.0f - std::exp(-tuning.jetAlign * std::max(lift, 0.2f) * dt));
  const Vector3 sag = Vector3Scale(Vector3Subtract(Vector3{0.0f, -kGravity, 0.0f},
                                                   Vector3Scale(path, -kGravity * path.y)),
                                   (1.0f - lift) * dt / speed);
  if (Vector3LengthSqr(sag) > 1e-12f) {
    const Vector3 sagged = Vector3Normalize(Vector3Add(path, sag));
    const Vector3 axisSag = Vector3CrossProduct(path, sagged);
    const float s = Vector3Length(axisSag);
    if (s > 1e-7f) body = rotateAttitude(body, Vector3Scale(axisSag, std::asin(std::min(s, 1.0f)) / (s * dt)), dt);
    path = sagged;
  }

  // ---- Speed ------------------------------------------------------------------
  player.throttle = std::clamp(player.throttle + axis(input.forward, input.back) * tuning.throttleRate * dt, 0.0f, 1.0f);
  player.airbrake = input.back && player.throttle <= 0.0f;
  const float turnG = speed * std::hypot(turnRate * cosPitch, pitchRate) / kGravity;
  const float drag = jetDrag() * speed * speed * (1.0f + (player.airbrake ? tuning.jetAirbrakeDrag : 0.0f));
  const float accel = tuning.jetThrustAccel * player.throttle - drag - kGravity * path.y - tuning.jetTurnBleed * turnG * turnG;
  speed = std::max(speed + accel * dt, kJetMinSpeed);

  player.body = body;
  player.velocity = Vector3Scale(path, speed);
}
