#pragma once

#include "raylib.h"

#include <span>

struct BoxObstacle {
  Vector3 center;
  Vector3 size;
  Color color;
};

std::span<const BoxObstacle> levelObstacles();
