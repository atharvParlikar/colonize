#pragma once

#include "raylib.h"

#include <cstdint>
#include <vector>

// Infinite, deterministic terrain: pure functions of world position, so chunks
// can be generated on any thread in any order.
float terrainHeight(float x, float z);
Vector3 terrainNormal(float x, float z);
float forestDensity(float x, float z);
// Coulomb friction coefficient of the ground surface (grass, rock or snow).
float surfaceFriction(float x, float z);

enum class TreeKind { Round, Conifer };

struct TreePlacement {
  Vector3 position;
  float scale;
  float yaw;
  float variation; // [0, 1] picks foliage color
  TreeKind kind;
  std::uint32_t seed;
};

// Appends every tree whose position lies in [x0, x1) x [z0, z1).
void collectTrees(float x0, float z0, float x1, float z1,
                  std::vector<TreePlacement> &out);
