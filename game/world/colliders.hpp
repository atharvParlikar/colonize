#pragma once

#include "raylib.h"
#include "world/level.hpp"

#include <span>

BoundingBox makeBoxBounds(Vector3 center, Vector3 size);
std::span<const BoundingBox> getLevelColliders();
