#pragma once

#include "combat/combat.hpp"

// Drones, tracers and sparks. Inside the 3D scene; alpha blends each round and
// spark between its last two steps, like the camera.
void drawCombat(const Combat &combat, const Camera3D &camera, float alpha);
