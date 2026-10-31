#pragma once

#include "raylib.h"

// GPU-generated grass (blades up close, tufts further out) driven by a
// GPU-rendered wind bitmap. Call after initScenery().
void initGrass();
void shutdownGrass();

// Renders the wind bitmap. Call once per frame before beginScene().
void updateGrassWind(const Camera3D &camera, float time);

// Draws blades and tufts; must run inside the 3D scene (between beginScene()
// and endScene()). `skiff` pushes the grass flat as it hovers overhead;
// `velocity` shrinks the grass range at speed (see motion.hpp).
void drawGrass(const Camera3D &camera, Vector3 skiff, Vector3 velocity, float time);
