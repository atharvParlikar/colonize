#include "collision/collision.hpp"

#include "world/colliders.hpp"
#include "world/terrain.hpp"

#include <algorithm>
#include <cmath>

using namespace engine;

float resolvePlayerCollisions(Player &player) {
  const Vector3 velocityBefore = player.velocity;

  // Penetration is measured before the push-out, otherwise a craft pressing
  // down onto a box would keep its velocity into it.
  const Capsule penetrating = makePlayerCapsule(player);
  Capsule capsule = penetrating;
  resolveCapsuleCollisions(capsule, player.position, getLevelColliders());

  for (const BoundingBox &collider : getLevelColliders()) {
    const CollisionInfo info = computeCapsuleAabbPenetration(penetrating, collider);
    if (!info.hit) {
      continue;
    }

    const float velocityAlongNormal =
        Vector3DotProduct(player.velocity, info.normal);
    if (velocityAlongNormal < 0.0f) {
      player.velocity = Vector3Subtract(
          player.velocity, Vector3Scale(info.normal, velocityAlongNormal));
    }
  }
  float impact =
      Vector3Length(Vector3Subtract(velocityBefore, player.velocity));

  const float groundY = terrainHeight(player.position.x, player.position.z);
  const float feetY = player.position.y - kEyeHeight;
  if (feetY < groundY) {
    player.position.y += groundY - feetY;
    const Vector3 normal = terrainNormal(player.position.x, player.position.z);
    const float into = Vector3DotProduct(player.velocity, normal);
    if (into < 0.0f) {
      impact = std::max(impact, -into);
      player.velocity = Vector3Subtract(player.velocity, Vector3Scale(normal, into));
    }
  }
  return impact;
}

GroundContact queryGround(const Player &player, float tolerance) {
  GroundContact contact;
  const bool onBox = isCapsuleGrounded(makePlayerCapsule(player), getLevelColliders(),
                                       std::min(tolerance, 0.05f));
  const float feetY = player.position.y - kEyeHeight;
  const float groundY = terrainHeight(player.position.x, player.position.z);
  const bool onTerrain = feetY - groundY < tolerance;

  contact.grounded = onBox || onTerrain;
  if (onBox) {
    contact.friction = kStoneFriction;
  } else if (onTerrain) {
    contact.normal = terrainNormal(player.position.x, player.position.z);
    contact.friction = surfaceFriction(player.position.x, player.position.z);
  }
  return contact;
}

void snapToGround(Player &player) {
  if (isCapsuleGrounded(makePlayerCapsule(player), getLevelColliders())) {
    return;
  }
  const float feetY = player.position.y - kEyeHeight;
  const float gap = feetY - terrainHeight(player.position.x, player.position.z);
  if (gap > 0.0f && gap < kGroundSnap) {
    player.position.y -= gap;
  }
}
