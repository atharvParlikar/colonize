#include "core/tuning.hpp"

float jetDrag() { return tuning.jetThrustAccel / (tuning.jetTopSpeed * tuning.jetTopSpeed); }
