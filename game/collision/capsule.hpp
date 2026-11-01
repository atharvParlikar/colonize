#pragma once

#include "core/constants.hpp"
#include "engine/collision/capsule.hpp"
#include "player/player.hpp"
#include "raylib.h"

// The player's body as an engine capsule, built from the eye position.
Vector3 playerFeetPosition(const Player &player);
engine::Capsule makePlayerCapsule(const Player &player);
engine::Capsule makePlayerCapsuleAtEyePosition(Vector3 eyePosition);
