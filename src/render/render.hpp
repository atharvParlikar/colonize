#pragma once

#include "player/player.hpp"
#include "raylib.h"

void renderWorld(const Camera3D &camera, const Player &player);
void drawCollisionDebug(const Player &player);
void drawHud(const Player &player, const Camera3D &camera);
