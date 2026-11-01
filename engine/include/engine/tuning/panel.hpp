#pragma once

#include "engine/tuning/tuning.hpp"

#include <span>
#include <vector>

namespace engine {

// In-game slider panel for a tuning table. While it is open the cursor is free;
// the game decides what that means for its own mouse input.
//   click / drag a slider    set the value
//   Shift + drag              fine adjust (1/10)
//   mouse wheel on a slider   nudge (Shift: finer)
//   right click a slider      back to its default
// One tab per group, in the order the groups first appear in the table.
// Save / Load use `file` in the working directory.
struct TuningPanelConfig {
  const char *title;      // e.g. "FLIGHT TUNING"
  const char *file;       // e.g. "tuning.txt"
  const char *fileHeader; // first-line comment of the save file
  std::span<const TuningParam> params;
};

class TuningPanel {
public:
  explicit TuningPanel(const TuningPanelConfig &config);

  bool isOpen() const { return open_; }
  void toggle();
  // Draws the panel and handles its mouse input. Call last in the 2D pass.
  // `readout` is a line of live values shown under the title (may be null).
  void draw(const char *readout);

private:
  void sliderRow(const TuningParam &p, float y);
  void setStatus(const char *text);

  TuningPanelConfig config_;
  std::vector<const char *> groups_;
  bool open_ = false;
  int group_ = 0;
  const TuningParam *dragging_ = nullptr;
  float dragLastX_ = 0.0f;
  const char *status_ = "";
  double statusTime_ = -10.0;
};

} // namespace engine
