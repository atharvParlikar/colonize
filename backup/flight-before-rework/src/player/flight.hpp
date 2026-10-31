#pragma once

#include "player/airframe.hpp"
#include "player/player.hpp"

// Mouse-aim flight law, after War Thunder's "instructor". The mouse moves an
// aim point (`Player.aimDir`, drawn as a circle on the HUD) around the cockpit,
// and the loop flies the nose onto it:
//   - far from the circle it rolls the lift side toward it, then pulls, so a
//     hard turn is a roll followed by a pull (loops, split-S, 180+ degree turns);
//   - near the circle it levels the wings and trims the nose so the flight path,
//     not just the nose, lands on it;
//   - pitch authority fades before the wing stalls, so a full pull is the
//     maximum-g turn rather than a stall.
// W spools the engine (idle, then turbo above 6000 rpm), S brakes, A / D roll
// by hand. The control surfaces take the torque first (authority scales with
// dynamic pressure) and the lift jets make up the rest; the lift jets also
// carry the weight below flying speed and fade out as the wing takes over.
Controls flightControl(Player &player, const PlayerInput &input, float dt);

// Main-jet thrust as a multiple of its 1x rating for an engine rpm: linear up
// to 1x at the turbo threshold, 2x at the turbo kick, 3x at the redline.
float engineThrust(float rpm);
