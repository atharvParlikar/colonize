#pragma once

#include "core/controls.hpp"
#include "player/player.hpp"

// Copies this frame's flight actions into input. Call after controls.update().
void samplePlayerInput(const Controls &controls, PlayerInput &input);
