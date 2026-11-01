#pragma once

#include "raylib.h"

#include <optional>

namespace engine {

// Sweep tests for fast, small things (bullets): does the segment from p0 to
// p1 touch the shape, and where first? The result is t in [0, 1] along the
// segment (hit point = p0 + (p1 - p0) * t); 0 when p0 is already inside.
std::optional<float> segmentSphere(Vector3 p0, Vector3 p1, Vector3 center, float radius);
std::optional<float> segmentAabb(Vector3 p0, Vector3 p1, BoundingBox box);

} // namespace engine
