#pragma once

#include "player/player.hpp"

// Hover is a game controller, not a flight model: the mouse looks around, WASD
// asks for a velocity relative to the look heading, Space / Shift climb and
// descend, and the velocity ramps to it (tuning.hoverAccel up, tuning.hoverBrake down), so
// letting go stops you. The engine holds the craft up; the lean you see is
// cosmetic. authority (0..1) scales the ramps while the jet hands over.
void stepHover(Player &player, const PlayerInput &input, float authority, float dt);

// Points the look where the jet's nose was, for a smooth hand-over.
void enterHover(Player &player);
