#include "player/aero.hpp"

#include "core/constants.hpp"
#include "raymath.h"

#include <algorithm>
#include <cmath>

namespace {

float smoothstep(float a, float b, float x) {
  const float t = std::clamp((x - a) / (b - a), 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

} // namespace

Attitude makeAttitude(Vector3 forward, Vector3 upHint) {
  Attitude a;
  a.forward = Vector3Normalize(forward);
  Vector3 right = Vector3CrossProduct(a.forward, upHint);
  if (Vector3LengthSqr(right) < 1e-6f) {
    right = Vector3CrossProduct(a.forward, Vector3{1.0f, 0.0f, 0.0f});
  }
  a.right = Vector3Normalize(right);
  a.up = Vector3CrossProduct(a.right, a.forward);
  return a;
}

Attitude windAttitude(Vector3 velocity, float alpha, float bank, const Attitude &previous) {
  const float speed = Vector3Length(velocity);
  if (speed < 2.0f) {
    return previous;
  }
  const Vector3 w = Vector3Scale(velocity, 1.0f / speed);

  Vector3 r0 = Vector3CrossProduct(w, kWorldUp);
  if (Vector3LengthSqr(r0) < 0.0025f) {
    // Near-vertical flight: keep the previous wing direction.
    r0 = Vector3Subtract(previous.right, Vector3Scale(w, Vector3DotProduct(previous.right, w)));
    if (Vector3LengthSqr(r0) < 1e-6f) {
      r0 = Vector3CrossProduct(w, Vector3{1.0f, 0.0f, 0.0f});
    }
  }
  r0 = Vector3Normalize(r0);
  const Vector3 u0 = Vector3CrossProduct(r0, w);

  const float cb = std::cos(bank);
  const float sb = std::sin(bank);
  const Vector3 r1 = Vector3Subtract(Vector3Scale(r0, cb), Vector3Scale(u0, sb));
  const Vector3 u1 = Vector3Add(Vector3Scale(u0, cb), Vector3Scale(r0, sb));

  const float ca = std::cos(alpha);
  const float sa = std::sin(alpha);
  Attitude out;
  out.forward = Vector3Add(Vector3Scale(w, ca), Vector3Scale(u1, sa));
  out.up = Vector3Subtract(Vector3Scale(u1, ca), Vector3Scale(w, sa));
  out.right = r1;
  return out;
}

float airDensity(float altitude) {
  return kAirDensitySeaLevel * std::exp(-std::max(altitude, 0.0f) / kAirScaleHeight);
}

void wingCoefficients(float alpha, float &cl, float &cd) {
  const float stall = kStallAngleDeg * DEG2RAD;
  const float absAlpha = std::fabs(alpha);

  // Attached flow: linear lift, induced drag on top of profile drag.
  const float clAttached = kLiftSlope * std::clamp(alpha, -stall, stall);
  const float cdAttached = kProfileDrag + clAttached * clAttached /
                                              (PI * kOswaldEfficiency * kAspectRatio);

  // Separated flow: flat-plate behaviour, valid all the way round the circle.
  const float clStalled = kFlatPlateLift * std::sin(2.0f * alpha);
  const float cdStalled = kProfileDrag + kFlatPlateDrag * std::sin(alpha) * std::sin(alpha);

  const float blend =
      absAlpha > 0.5f * PI
          ? 1.0f
          : smoothstep(stall, stall + kStallBlendDeg * DEG2RAD, absAlpha);
  cl = clAttached + (clStalled - clAttached) * blend;
  cd = cdAttached + (cdStalled - cdAttached) * blend;
}

AeroResult computeWingsuitAero(Vector3 velocity, const Attitude &attitude, float altitude) {
  AeroResult out;
  const float speed = Vector3Length(velocity);
  out.airspeed = speed;
  if (speed < 0.5f) {
    return out;
  }

  const Vector3 vhat = Vector3Scale(velocity, 1.0f / speed);
  const float vf = Vector3DotProduct(velocity, attitude.forward);
  const float vu = Vector3DotProduct(velocity, attitude.up);
  const float vr = Vector3DotProduct(velocity, attitude.right);

  out.alpha = std::atan2(-vu, vf);
  out.beta = std::asin(std::clamp(vr / speed, -1.0f, 1.0f));
  wingCoefficients(out.alpha, out.cl, out.cd);
  out.stalled = std::fabs(out.alpha) > kStallAngleDeg * DEG2RAD;

  const float q = 0.5f * airDensity(altitude) * speed * speed;

  // Lift acts perpendicular to the flow, on the wing-normal side.
  Vector3 liftDir = Vector3Subtract(
      attitude.up, Vector3Scale(vhat, Vector3DotProduct(attitude.up, vhat)));
  if (Vector3LengthSqr(liftDir) > 1e-6f) {
    liftDir = Vector3Normalize(liftDir);
  } else {
    liftDir = Vector3Zero();
  }

  const Vector3 lift = Vector3Scale(liftDir, q * kWingArea * out.cl);
  const Vector3 drag = Vector3Scale(vhat, -q * kWingArea * out.cd);
  const Vector3 side = Vector3Scale(attitude.right, -q * kWingArea * kSideForceSlope * out.beta);
  out.force = Vector3Add(Vector3Add(lift, drag), side);
  return out;
}

Vector3 computeBluffBodyDrag(Vector3 velocity, float altitude) {
  const float speed = Vector3Length(velocity);
  return Vector3Scale(velocity, -0.5f * airDensity(altitude) * kFallingDragArea * speed);
}
