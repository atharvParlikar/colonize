#pragma once

#include "combat/combat.hpp"
#include "player/player.hpp"
#include "raylib.h"

void renderWorld(const Camera3D &camera, const Player &player, const Combat &combat, float alpha);
void drawCollisionDebug(const Player &player);
void drawHud(const Player &player, const Combat &combat, const Camera3D &camera);
