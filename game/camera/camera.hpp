#pragma once

#include "core/constants.hpp"
#include "player/player.hpp"
#include "raylib.h"

struct View {
  Vector3 position;
  Vector3 forward;
  Vector3 up;
  float fovy;
};

// Blends the previous and current physics states for smooth rendering between
// fixed steps. alpha in [0, 1] is how far we are into the next step.
View computeView(const Player &previous, const Player &current, float alpha);
Camera3D makeCamera(const View &view);
void syncCamera(Camera3D &camera, const View &view);
