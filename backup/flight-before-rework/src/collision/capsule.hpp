#pragma once

#include "core/constants.hpp"
#include "player/player.hpp"
#include "raylib.h"

struct Capsule {
  Vector3 start;
  Vector3 end;
  float radius;
};

Vector3 playerFeetPosition(const Player &player);
Capsule makePlayerCapsule(const Player &player);
Capsule makePlayerCapsuleAtEyePosition(Vector3 eyePosition);
