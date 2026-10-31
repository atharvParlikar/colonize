#pragma once

#include "player/airframe.hpp"
#include "player/player.hpp"

// The pilot's input is a velocity: WASD along the heading, Space/Shift for
// climb and descent, mouse for heading. This turns it into jet commands:
//   velocity error -> wanted thrust vector -> wanted tilt -> wanted rates -> torque -> mixer.
Controls hoverControl(const Player &player, const PlayerInput &input);

// Rate command for an angle error: proportional near the target, but never
// faster than the torque limit can brake from (v^2 = 2 a e).
float shapedRate(float error, float gain, float maxRate, float maxAccel);

