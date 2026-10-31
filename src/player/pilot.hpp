#pragma once

#include "player/airframe.hpp"
#include "player/player.hpp"

// One control law for every speed. The pilot says where to go and how fast:
//   - the mouse moves the aim point (Player.aimDir), the craft goes where it points;
//     flying it is a spring-loaded stick: a short lead that eases back onto the path;
//   - W / S move the speed command along it (below kCruiseLatch, letting go
//     brings it to a hover; above it the speed holds);
//   - A / D strafe when slow and roll when fast, Space / Shift climb and descend
//     when slow.
// That becomes a wanted acceleration, then the force the airframe must make
// (gravity included), then:
//   hover  - tilt the body toward that force, face the aim, nozzle down;
//   flight - bank so the lift points along the force's part across the flight
//            path, raise the nose by the angle of attack the wing needs for it,
//            nozzle aft;
// blended by airspeed. Whatever the wing and hull do not provide, the engine
// makes up through the nozzle, so the craft follows the command closely and the
// flight path, not just the nose, lands on the aim point.
AirframeCommand pilotControl(Player &player, const PlayerInput &input, float dt);

// Rate command for an angle error: proportional near the target, but never
// faster than maxAccel can brake from (v^2 = 2 a e).
float shapedRate(float error, float gain, float maxRate, float maxAccel);
