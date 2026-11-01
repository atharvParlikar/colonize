#pragma once

#include "raylib.h"

void initScenery();
// Blocks (showing a loading screen) until the terrain around the camera is built.
void preloadScenery(const Camera3D &camera);
void shutdownScenery();

// Frame lifecycle: beginScene() opens the offscreen target + 3D mode and draws
// the sky; drawLandscape() draws terrain and trees with the world shader;
// endScene() closes 3D mode and paints the target to the window through the
// brush filter.
void beginScene(const Camera3D &camera);
void drawLandscape();
void endScene();

Shader &worldShader();

// Terrain albedo at a surface point; the grass samples it so blades match the
// ground they grow from.
Color terrainSurfaceColor(float x, float y, float z, Vector3 normal);
