#pragma once

#include "raylib.h"

#include <span>

namespace engine {

// A segment with a radius: the usual shape for a character or craft body.
struct Capsule {
  Vector3 start;
  Vector3 end;
  float radius;
};

struct CollisionInfo {
  Vector3 normal{}; // points out of the box, toward the capsule
  float depth = 0.0f;
  bool hit = false;
};

bool intersectsCapsuleAabb(const Capsule &capsule, BoundingBox box);
CollisionInfo computeCapsuleAabbPenetration(const Capsule &capsule, BoundingBox box);
// Pushes the capsule out of every box. `anchor` (e.g. the eye position the
// capsule was built from) is moved by the same amount.
void resolveCapsuleCollisions(Capsule &capsule, Vector3 &anchor, std::span<const BoundingBox> colliders);
bool isCapsuleGrounded(const Capsule &capsule, std::span<const BoundingBox> colliders, float tolerance = 0.05f);

} // namespace engine
