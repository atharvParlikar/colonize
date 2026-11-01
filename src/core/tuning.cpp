#include "core/tuning.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>

float jetDrag() { return tuning.jetThrustAccel / (tuning.jetTopSpeed * tuning.jetTopSpeed); }

bool saveTuning(const char *path) {
  FILE *file = std::fopen(path, "w");
  if (!file) return false;
  std::fprintf(file, "# SKIFF flight tuning (written by the in-game panel). key = value\n");
  for (const TuningParam &p : kTuningParams) std::fprintf(file, "%s = %.6g\n", p.name, tuning.*p.field);
  std::fclose(file);
  return true;
}

bool loadTuning(const char *path) {
  FILE *file = std::fopen(path, "r");
  if (!file) return false;
  char line[256];
  while (std::fgets(line, sizeof line, file)) {
    char key[128];
    float value = 0.0f;
    if (line[0] == '#' || std::sscanf(line, " %127[^= ] = %f", key, &value) != 2) continue;
    for (const TuningParam &p : kTuningParams) {
      if (std::strcmp(p.name, key) == 0) tuning.*p.field = std::clamp(value, p.lo, p.hi);
    }
  }
  std::fclose(file);
  return true;
}
