#pragma once

#include "player/player.hpp"

// The jet is kinematic but has momentum:
//   - the mouse deflects a spring-loaded stick (tuning.stickReturnTau): X turns the
//     heading and the jet banks into the turn on its own, Y pitches about the
//     wing; A / D roll by hand (the bank holds, then eases back); sideways
//     wobble on a pitch swipe is ignored (tuning.jetPitchWobble) so the wings
//     stay level;
//   - the turn rate is capped by tuning.jetMaxRateDeg and by the g the airframe
//     allows at this speed, which a blacked-out pilot cannot reach (soft loss
//     of control);
//   - the flight path follows the nose with a short lag;
//   - speed is one number: throttle (W / S) against drag, gravity along the
//     path, and the bleed a hard turn costs; S at idle opens the airbrake;
//   - below tuning.jetStallSpeed the wing lets the path and the nose sag.
// authority (0..1) scales the stick while the nozzles swing over.
void stepJet(Player &player, const PlayerInput &input, float authority, float dt);

// Hand-over from hover: nose along the look, throttle set to hold the speed.
void enterJet(Player &player);
