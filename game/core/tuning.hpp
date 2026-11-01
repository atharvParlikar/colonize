#pragma once

#include "engine/tuning/tuning.hpp"

// Live flight tuning. Every number the hover, jet and g models are tuned by
// lives here, at run time, so the in-game panel (Tab) can change it while you
// fly. One list drives the struct, the panel and the save file:
//   X(group, name, default, min, max, label, unit)
// The defaults are the shipped feel; tuning.txt (Save in the panel) overrides
// them at startup. Save / load / the panel itself live in engine/tuning.
#define FLIGHT_TUNING(X)                                                                                     \
  X("Hover", lookSensitivity, 0.0025f, 0.0005f, 0.008f, "Look sensitivity", "rad/count")                    \
  X("Hover", lookMaxPitchDeg, 85.0f, 30.0f, 89.0f, "Look pitch limit", "deg")                               \
  X("Hover", hoverMaxSpeed, 40.0f, 10.0f, 80.0f, "Forward speed (W)", "m/s")                                \
  X("Hover", hoverReverseSpeed, 12.0f, 2.0f, 40.0f, "Reverse speed (S)", "m/s")                             \
  X("Hover", hoverStrafeSpeed, 18.0f, 2.0f, 50.0f, "Strafe speed (A/D)", "m/s")                             \
  X("Hover", hoverClimbSpeed, 12.0f, 2.0f, 40.0f, "Climb speed (Space)", "m/s")                             \
  X("Hover", hoverDescendSpeed, 10.0f, 2.0f, 40.0f, "Descend speed (Shift)", "m/s")                         \
  X("Hover", hoverAccel, 22.0f, 2.0f, 80.0f, "Acceleration", "m/s2")                                        \
  X("Hover", hoverBrake, 32.0f, 2.0f, 100.0f, "Braking (let go)", "m/s2")                                   \
  X("Hover", hoverVerticalAccel, 18.0f, 2.0f, 80.0f, "Vertical acceleration", "m/s2")                       \
  X("Hover", hoverTurnRate, 10.0f, 1.0f, 30.0f, "Body follows look", "1/s")                                 \
  X("Hover", hoverTiltPerAccel, 0.9f, 0.0f, 3.0f, "Lean per acceleration", "deg/m/s2")                      \
  X("Hover", hoverMaxTiltDeg, 18.0f, 0.0f, 45.0f, "Max lean", "deg")                                        \
  X("Conversion", convertSpeed, 30.0f, 5.0f, 80.0f, "Hover -> jet gate (above)", "m/s")                     \
  X("Conversion", convertBackSpeed, 70.0f, 20.0f, 170.0f, "Jet -> hover gate (below)", "m/s")               \
  X("Conversion", convertTime, 1.5f, 0.2f, 5.0f, "Nozzle swing time", "s")                                  \
  X("Jet", stickSensitivity, 0.004f, 0.0005f, 0.02f, "Stick sensitivity", "/count")                         \
  X("Jet", stickReturnTau, 0.25f, 0.02f, 3.0f, "Stick spring-back time", "s")                               \
  X("Jet", jetPitchWobble, 0.4f, 0.0f, 0.9f, "Pitch: ignore sideways below", "x pitch")                     \
  X("Jet", jetMaxRateDeg, 80.0f, 10.0f, 200.0f, "Max turn / pitch rate", "deg/s")                           \
  X("Jet", jetMaxG, 10.0f, 2.0f, 20.0f, "Max g (airframe)", "g")                                            \
  X("Jet", jetRollRate, 3.5f, 0.5f, 8.0f, "Hand roll rate (A/D)", "rad/s")                                  \
  X("Jet", jetMaxAutoRollRate, 4.0f, 0.5f, 8.0f, "Auto-bank roll rate", "rad/s")                            \
  X("Jet", jetRollGain, 6.0f, 0.5f, 20.0f, "Auto-bank stiffness", "1/s")                                    \
  X("Jet", jetMaxAutoBankDeg, 85.0f, 0.0f, 89.0f, "Max auto-bank", "deg")                                   \
  X("Jet", rollTrimReturnTau, 2.0f, 0.1f, 10.0f, "Hand bank eases back", "s")                               \
  X("Jet", jetAlign, 6.0f, 0.5f, 30.0f, "Path follows nose", "1/s")                                         \
  X("Jet", jetThrustAccel, 18.0f, 2.0f, 60.0f, "Thrust (full throttle)", "m/s2")                            \
  X("Jet", jetTopSpeed, 170.0f, 60.0f, 400.0f, "Top level speed", "m/s")                                    \
  X("Jet", jetAirbrakeDrag, 2.0f, 0.0f, 10.0f, "Airbrake (x drag)", "")                                     \
  X("Jet", jetTurnBleed, 0.14f, 0.0f, 1.0f, "Turn bleed", "m/s2/g2")                                        \
  X("Jet", jetStallSpeed, 45.0f, 5.0f, 100.0f, "Stall speed", "m/s")                                        \
  X("Jet", throttleRate, 0.7f, 0.1f, 3.0f, "Throttle rate (W/S)", "/s")                                     \
  X("g", gOnset, 5.0f, 1.0f, 12.0f, "Dose builds above", "g")                                               \
  X("g", gDoseExponent, 1.2f, 0.5f, 3.0f, "Dose curve exponent", "")                                        \
  X("g", gRecover, 4.0f, 0.0f, 10.0f, "Dose drains below", "g")                                             \
  X("g", gRecoverRate, 3.0f, 0.1f, 10.0f, "Dose drain rate", "/s")                                          \
  X("g", greyoutDose, 2.0f, 0.1f, 20.0f, "Greyout starts at dose", "")                                      \
  X("g", tunnelDose, 5.0f, 0.2f, 30.0f, "Tunnel starts at dose", "")                                        \
  X("g", blackoutDose, 12.0f, 0.5f, 40.0f, "Black at dose", "")                                             \
  X("g", gLocMaxG, 6.0f, 1.0f, 20.0f, "Blacked-out max g", "g")                                             \
  X("g", gLocStickLoss, 0.4f, 0.0f, 1.0f, "Blacked-out stick loss", "")                                     \
  X("g", negGOnset, 2.0f, 0.5f, 8.0f, "Redout builds below -", "g")                                         \
  X("g", negGRecover, 1.0f, 0.0f, 5.0f, "Redout drains above -", "g")                                       \
  X("g", negGDoseRate, 1.5f, 0.1f, 5.0f, "Redout dose rate", "x")                                           \
  X("g", redoutStartDose, 1.0f, 0.1f, 10.0f, "Redout starts at dose", "")                                   \
  X("g", redoutDose, 3.0f, 0.2f, 20.0f, "Full redout at dose", "")                                          \
  X("g", shakeStartG, 4.0f, 0.0f, 15.0f, "Shake starts at", "g")                                            \
  X("g", shakeFullG, 11.0f, 1.0f, 20.0f, "Full shake at", "g")                                              \
  X("g", gSmoothTau, 0.08f, 0.01f, 0.5f, "g reading smoothing", "s")

struct FlightTuning {
#define FLIGHT_TUNING_FIELD(group, name, def, lo, hi, label, unit) float name = def;
  FLIGHT_TUNING(FLIGHT_TUNING_FIELD)
#undef FLIGHT_TUNING_FIELD
};

// The live values. Tests and a fresh start use the defaults.
inline FlightTuning tuning;

// The table the engine's tuning panel and save file work from.
inline constexpr engine::TuningParam kFlightTuningParams[] = {
#define FLIGHT_TUNING_PARAM(group, name, def, lo, hi, label, unit) {group, #name, label, unit, &tuning.name, def, lo, hi},
    FLIGHT_TUNING(FLIGHT_TUNING_PARAM)
#undef FLIGHT_TUNING_PARAM
};

// Saved by the panel (Tab), in the working directory; loaded at startup.
constexpr const char *kTuningFile = "tuning.txt";

// Drag coefficient that makes full throttle top out at jetTopSpeed.
float jetDrag();
