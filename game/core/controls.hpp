#pragma once

#include "engine/input/actions.hpp"

// Every action the player can take. Add one here and give it a default
// binding in controls.cpp.
enum class Action {
  Forward,      // hover: fly forward / jet: throttle up
  Back,         // hover: reverse / jet: throttle down, airbrake at idle
  Left,         // hover: strafe / jet: roll
  Right,
  Ascend,
  Descend,
  Convert,      // arm the hover <-> jet conversion
  Fire,         // cannon
  TuningPanel,  // open / close the live tuning panel
  Count
};

using Controls = engine::ActionMap<Action>;

// The default key layout.
void bindDefaultControls(Controls &controls);
