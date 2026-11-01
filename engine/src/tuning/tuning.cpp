#include "engine/tuning/tuning.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace engine {

bool saveTuning(const char *path, std::span<const TuningParam> params, const char *header) {
  FILE *file = std::fopen(path, "w");
  if (!file) return false;
  std::fprintf(file, "# %s. key = value\n", header);
  for (const TuningParam &p : params) std::fprintf(file, "%s = %.6g\n", p.name, *p.value);
  std::fclose(file);
  return true;
}

bool loadTuning(const char *path, std::span<const TuningParam> params) {
  FILE *file = std::fopen(path, "r");
  if (!file) return false;
  char line[256];
  while (std::fgets(line, sizeof line, file)) {
    char key[128];
    float value = 0.0f;
    if (line[0] == '#' || std::sscanf(line, " %127[^= ] = %f", key, &value) != 2) continue;
    for (const TuningParam &p : params) {
      if (std::strcmp(p.name, key) == 0) *p.value = std::clamp(value, p.lo, p.hi);
    }
  }
  std::fclose(file);
  return true;
}

void resetTuning(std::span<const TuningParam> params, const char *group) {
  for (const TuningParam &p : params) {
    if (group == nullptr || std::strcmp(p.group, group) == 0) *p.value = p.def;
  }
}

} // namespace engine
