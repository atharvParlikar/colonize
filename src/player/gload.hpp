#pragma once

// What the pilot's body makes of the load. g is the felt load along the
// canopy axis (+1 sitting still, positive pushes you into the seat). Positive
// load above tuning.gOnset builds a dose, faster the harder you pull (6 g for ~2-3 s
// or 10 g for a fraction of a second starts the greyout); below tuning.gRecover it
// drains. Negative load builds its own dose (redout). The dose drives the
// screen, audio and camera effects, and in the jet a soft loss of control.
struct GLoad {
  float g = 1.0f;          // felt load, smoothed (g)
  float dose = 0.0f;       // positive-g stress
  float negDose = 0.0f;    // negative-g stress
  float greyout = 0.0f;    // 0..1: colour drains
  float blackout = 0.0f;   // 0..1: tunnel closes to black
  float redout = 0.0f;     // 0..1: red-out from negative g
  float shake = 0.0f;      // 0..1: camera shake from the load itself
};

// gRaw: this step's felt load (g). Pure, so it can be tested on its own.
void stepGLoad(GLoad &load, float gRaw, float dt);
