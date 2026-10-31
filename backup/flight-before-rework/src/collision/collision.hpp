#pragma once

#include "collision/capsule.hpp"
#include "player/player.hpp"
#include "raylib.h"

#include <span>

struct CollisionInfo {
  Vector3 normal{};
  float depth = 0.0f;
  bool hit = false;
};

bool intersectsCapsuleAabb(const Capsule &capsule, BoundingBox box);
CollisionInfo computeCapsuleAabbPenetration(const Capsule &capsule,
                                            BoundingBox box);
void resolveCapsuleCollisions(Capsule &capsule, Vector3 &eyePosition,
                              std::span<const BoundingBox> colliders);
bool isCapsuleGrounded(const Capsule &capsule,
                       std::span<const BoundingBox> colliders,
                       float tolerance = 0.05f);

struct GroundContact {
  bool grounded = false;
  Vector3 normal{0.0f, 1.0f, 0.0f};
  float friction = 0.6f;
};

// Is the player standing on (or within tolerance of) a surface, and which one?
GroundContact queryGround(const Player &player, float tolerance);
// Pushes the player out of geometry and removes velocity into surfaces.
// Returns the speed lost to impacts this call (m/s).
float resolvePlayerCollisions(Player &player);
// Keeps a walking player glued to terrain across small drops.
void snapToGround(Player &player);
