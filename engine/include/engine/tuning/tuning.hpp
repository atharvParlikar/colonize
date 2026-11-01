#pragma once

#include <span>

namespace engine {

// One live-tunable number. The game owns the float; the engine only knows
// where it is and its range, which is enough to draw a slider for it and to
// save / load it. A table of these is the engine's whole view of a tuning set.
struct TuningParam {
  const char *group; // tab in the panel
  const char *name;  // key in the save file
  const char *label; // shown in the panel
  const char *unit;
  float *value;
  float def, lo, hi;
};

// key = value lines, one per parameter. `header` is written as a # comment.
bool saveTuning(const char *path, std::span<const TuningParam> params, const char *header);
// Unknown keys are ignored; values are clamped to their range.
bool loadTuning(const char *path, std::span<const TuningParam> params);
// Back to defaults: one group, or every parameter when group is nullptr.
void resetTuning(std::span<const TuningParam> params, const char *group = nullptr);

} // namespace engine
