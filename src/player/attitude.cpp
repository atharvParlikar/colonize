#include "player/attitude.hpp"

#include <algorithm>
#include <cmath>

Attitude makeAttitude(Vector3 forward, Vector3 upHint) {
  Attitude a;
  a.forward = Vector3Normalize(forward);
  Vector3 right = Vector3CrossProduct(a.forward, upHint);
  if (Vector3LengthSqr(right) < 1e-6f) right = Vector3CrossProduct(a.forward, Vector3{1.0f, 0.0f, 0.0f});
  if (Vector3LengthSqr(right) < 1e-6f) right = Vector3CrossProduct(a.forward, Vector3{0.0f, 0.0f, 1.0f});
  a.right = Vector3Normalize(right);
  a.up = Vector3CrossProduct(a.right, a.forward);
  return a;
}

Attitude rotateAttitude(const Attitude &a, Vector3 omega, float dt) {
  const float rate = Vector3Length(omega);
  if (rate < 1e-7f) return a;
  const Vector3 axis = Vector3Scale(omega, 1.0f / rate);
  return makeAttitude(Vector3RotateByAxisAngle(a.forward, axis, rate * dt),
                      Vector3RotateByAxisAngle(a.up, axis, rate * dt));
}

Attitude blendAttitude(const Attitude &from, const Attitude &to, float k) {
  const Vector3 forward = Vector3Lerp(from.forward, to.forward, k);
  const Vector3 up = Vector3Lerp(from.up, to.up, k);
  if (Vector3LengthSqr(forward) < 1e-6f || Vector3LengthSqr(up) < 1e-6f) return k < 0.5f ? from : to;
  return makeAttitude(forward, up);
}

float bankOf(const Attitude &a) { return std::atan2(-a.right.y, a.up.y); }

float pitchOf(const Attitude &a) { return std::asin(std::clamp(a.forward.y, -1.0f, 1.0f)); }

float headingOf(Vector3 dir) { return std::atan2(dir.x, -dir.z); }

Vector3 directionFrom(float heading, float pitch) {
  return {std::sin(heading) * std::cos(pitch), std::sin(pitch), -std::cos(heading) * std::cos(pitch)};
}

float smoothstep(float a, float b, float x) {
  const float t = std::clamp((x - a) / (b - a), 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

float wrapAngle(float a) { return std::remainder(a, 2.0f * PI); }
