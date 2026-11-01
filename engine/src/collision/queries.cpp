#include "engine/collision/queries.hpp"

#include "raymath.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace engine {

std::optional<float> segmentSphere(Vector3 p0, Vector3 p1, Vector3 center, float radius) {
  const Vector3 d = Vector3Subtract(p1, p0);
  const Vector3 m = Vector3Subtract(p0, center);
  const float c = Vector3DotProduct(m, m) - radius * radius;
  if (c <= 0.0f) return 0.0f; // starts inside

  // |m + t d|^2 = r^2  ->  a t^2 + 2 b t + c = 0
  const float a = Vector3DotProduct(d, d);
  const float b = Vector3DotProduct(m, d);
  if (a < 1e-12f || b >= 0.0f) return std::nullopt; // not moving, or moving away
  const float disc = b * b - a * c;
  if (disc < 0.0f) return std::nullopt;
  const float t = (-b - std::sqrt(disc)) / a;
  if (t > 1.0f) return std::nullopt;
  return std::max(t, 0.0f);
}

std::optional<float> segmentAabb(Vector3 p0, Vector3 p1, BoundingBox box) {
  // Slab method: clip [0, 1] against each axis' pair of planes.
  const float start[3] = {p0.x, p0.y, p0.z};
  const float delta[3] = {p1.x - p0.x, p1.y - p0.y, p1.z - p0.z};
  const float lo[3] = {box.min.x, box.min.y, box.min.z};
  const float hi[3] = {box.max.x, box.max.y, box.max.z};
  float tEnter = 0.0f, tExit = 1.0f;
  for (int i = 0; i < 3; ++i) {
    if (std::fabs(delta[i]) < 1e-12f) {
      if (start[i] < lo[i] || start[i] > hi[i]) return std::nullopt; // parallel and outside
      continue;
    }
    float t0 = (lo[i] - start[i]) / delta[i];
    float t1 = (hi[i] - start[i]) / delta[i];
    if (t0 > t1) std::swap(t0, t1);
    tEnter = std::max(tEnter, t0);
    tExit = std::min(tExit, t1);
    if (tEnter > tExit) return std::nullopt;
  }
  return tEnter;
}

} // namespace engine
