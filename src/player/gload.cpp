#include "player/gload.hpp"

#include "core/constants.hpp"
#include "player/attitude.hpp"

#include <algorithm>
#include <cmath>

void stepGLoad(GLoad &load, float gRaw, float dt) {
  load.g += (gRaw - load.g) * (1.0f - std::exp(-dt / tuning.gSmoothTau));
  const float g = load.g;

  // Positive g: the dose builds as (g - onset)^1.2 per second (1/s at 6 g,
  // ~7/s at 10 g), drains below tuning.gRecover, and holds in between.
  if (g > tuning.gOnset) {
    load.dose += std::pow(g - tuning.gOnset, tuning.gDoseExponent) * dt;
  } else if (g < tuning.gRecover) {
    load.dose -= tuning.gRecoverRate * dt;
  }
  load.dose = std::clamp(load.dose, 0.0f, tuning.blackoutDose + 1.0f);

  // Negative g: redout, quicker to come on (the body tolerates less of it).
  if (g < -tuning.negGOnset) {
    load.negDose += tuning.negGDoseRate * std::pow(-g - tuning.negGOnset, tuning.gDoseExponent) * dt;
  } else if (g > -tuning.negGRecover) {
    load.negDose -= tuning.gRecoverRate * dt;
  }
  load.negDose = std::clamp(load.negDose, 0.0f, tuning.redoutDose + 1.0f);

  load.greyout = smoothstep(tuning.greyoutDose, tuning.tunnelDose, load.dose);
  load.blackout = smoothstep(tuning.tunnelDose, tuning.blackoutDose, load.dose);
  load.redout = smoothstep(tuning.redoutStartDose, tuning.redoutDose, load.negDose);
  load.shake = std::max(smoothstep(tuning.shakeStartG, tuning.shakeFullG, g), smoothstep(tuning.negGOnset, tuning.negGOnset + 3.0f, -g));
}
