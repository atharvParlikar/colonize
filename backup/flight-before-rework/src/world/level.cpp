#include "world/colliders.hpp"
#include "world/level.hpp"

#include "raymath.h"

#include <array>

namespace {

constexpr BoxObstacle kObstacles[] = {
    {{0.0f, 0.0f, 0.0f}, {32.0f, 0.1f, 32.0f}, {198, 188, 168, 255}},
    {{0.0f, 1.0f, 0.0f}, {2.0f, 2.0f, 2.0f}, RED},
};

BoundingBox kColliderBounds[std::size(kObstacles)]{};

} // namespace

std::span<const BoxObstacle> levelObstacles() { return kObstacles; }

BoundingBox makeBoxBounds(const Vector3 center, const Vector3 size) {
  const Vector3 half = Vector3Scale(size, 0.5f);
  return {Vector3Subtract(center, half), Vector3Add(center, half)};
}

std::span<const BoundingBox> getLevelColliders() {
  static bool initialized = false;
  if (!initialized) {
    for (std::size_t i = 0; i < std::size(kObstacles); ++i) {
      kColliderBounds[i] =
          makeBoxBounds(kObstacles[i].center, kObstacles[i].size);
    }
    initialized = true;
  }
  return kColliderBounds;
}
