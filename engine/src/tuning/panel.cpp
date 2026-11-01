#include "engine/tuning/panel.hpp"

#include "raylib.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace engine {

namespace {

constexpr Color kCyan{90, 230, 255, 255};
constexpr Color kCyanDim{40, 120, 140, 255};
constexpr Color kAmber{255, 176, 60, 255};
constexpr Color kWhite{225, 245, 250, 255};
constexpr Color kBack{4, 14, 22, 228};

constexpr float kX = 20.0f, kY = 96.0f, kW = 600.0f;
constexpr float kRowH = 25.0f;
constexpr float kLabelW = 205.0f, kSliderW = 220.0f;

Color fade(Color c, float a) {
  c.a = static_cast<unsigned char>(std::clamp(a, 0.0f, 1.0f) * 255.0f);
  return c;
}

void label(const char *str, float x, float y, float size, Color color) {
  DrawTextEx(GetFontDefault(), str, {x, y}, size, 1.0f, color);
}

int decimals(const TuningParam &p) {
  const float range = p.hi - p.lo;
  return range < 0.05f ? 4 : range < 1.5f ? 3 : range < 15.0f ? 2 : 1;
}

bool button(const char *text, float x, float y, float w, bool active = false) {
  const Rectangle r{x, y, w, 24.0f};
  const bool hover = CheckCollisionPointRec(GetMousePosition(), r);
  DrawRectangleRec(r, active ? fade(kCyan, 0.28f) : (hover ? fade(kCyan, 0.16f) : fade(kCyanDim, 0.18f)));
  DrawRectangleLinesEx(r, 1.0f, active ? kCyan : kCyanDim);
  const float tw = MeasureTextEx(GetFontDefault(), text, 14, 1.0f).x;
  label(text, x + (w - tw) * 0.5f, y + 5, 14, active ? kWhite : kCyan);
  return hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

} // namespace

TuningPanel::TuningPanel(const TuningPanelConfig &config) : config_(config) {
  for (const TuningParam &p : config_.params) {
    const bool known = std::any_of(groups_.begin(), groups_.end(),
                                   [&](const char *g) { return std::strcmp(g, p.group) == 0; });
    if (!known) groups_.push_back(p.group);
  }
}

void TuningPanel::toggle() {
  open_ = !open_;
  dragging_ = nullptr;
  if (open_) {
    EnableCursor();
  } else {
    DisableCursor();
  }
}

void TuningPanel::setStatus(const char *text) {
  status_ = text;
  statusTime_ = GetTime();
}

// One slider row; handles its own input.
void TuningPanel::sliderRow(const TuningParam &p, float y) {
  float &value = *p.value;
  const float sx = kX + 14 + kLabelW;
  const Rectangle bar{sx, y + 7, kSliderW, 10};
  const Rectangle hit{sx - 4, y, kSliderW + 8, kRowH - 2};
  const Vector2 mouse = GetMousePosition();
  const bool hover = CheckCollisionPointRec(mouse, hit);
  const bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
  const float range = p.hi - p.lo;

  if (hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
    dragging_ = &p;
    dragLastX_ = mouse.x;
    if (!shift) value = p.lo + std::clamp((mouse.x - bar.x) / bar.width, 0.0f, 1.0f) * range;
  }
  if (dragging_ == &p && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
    if (shift) {
      value += (mouse.x - dragLastX_) / bar.width * range * 0.1f; // fine
    } else {
      value = p.lo + std::clamp((mouse.x - bar.x) / bar.width, 0.0f, 1.0f) * range;
    }
    dragLastX_ = mouse.x;
  }
  if (hover && IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) value = p.def;
  const float wheel = hover ? GetMouseWheelMove() : 0.0f;
  if (wheel != 0.0f) value += wheel * range * (shift ? 0.002f : 0.01f);
  value = std::clamp(value, p.lo, p.hi);

  const bool changed = std::fabs(value - p.def) > range * 1e-4f;
  if (hover || dragging_ == &p) DrawRectangle(static_cast<int>(kX + 6), static_cast<int>(y), static_cast<int>(kW - 12),
                                              static_cast<int>(kRowH - 2), fade(kCyan, 0.06f));
  label(p.label, kX + 14, y + 6, 14, changed ? kAmber : kWhite);

  // Bar: track, fill, default tick, knob.
  const float t = (value - p.lo) / range, td = (p.def - p.lo) / range;
  DrawRectangleRec(bar, fade(kCyanDim, 0.35f));
  DrawRectangle(static_cast<int>(bar.x), static_cast<int>(bar.y), static_cast<int>(bar.width * t),
                static_cast<int>(bar.height), changed ? fade(kAmber, 0.8f) : fade(kCyan, 0.7f));
  DrawLineEx({bar.x + bar.width * td, bar.y - 3}, {bar.x + bar.width * td, bar.y + bar.height + 3}, 2.0f, kWhite);
  DrawCircleV({bar.x + bar.width * t, bar.y + bar.height * 0.5f}, 6.0f, changed ? kAmber : kCyan);

  const char *text = TextFormat("%.*f", decimals(p), value);
  label(text, sx + kSliderW + 14, y + 6, 14, changed ? kAmber : kWhite);
  label(p.unit, sx + kSliderW + 84, y + 6, 12, kCyanDim);
}

void TuningPanel::draw(const char *readout) {
  if (!open_) return;
  if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) dragging_ = nullptr;

  const float kH = static_cast<float>(GetScreenHeight()) - 116.0f;
  DrawRectangle(static_cast<int>(kX), static_cast<int>(kY), static_cast<int>(kW), static_cast<int>(kH), kBack);
  DrawRectangleLinesEx({kX, kY, kW, kH}, 1.0f, kCyanDim);
  label(config_.title, kX + 14, kY + 10, 20, kCyan);
  label("Tab: close   drag: set   Shift: fine   wheel: nudge   right click: default", kX + 14, kY + 34, 12,
        kCyanDim);
  if (readout != nullptr) label(readout, kX + 14, kY + 52, 13, kWhite);

  // Group tabs.
  const int groupCount = static_cast<int>(groups_.size());
  if (groupCount == 0) return;
  const float tabW = (kW - 28 - (groupCount - 1) * 6) / groupCount;
  for (int i = 0; i < groupCount; ++i) {
    if (button(groups_[i], kX + 14 + i * (tabW + 6), kY + 74, tabW, i == group_)) group_ = i;
  }
  const char *groupName = groups_[group_];

  float y = kY + 108;
  for (const TuningParam &p : config_.params) {
    if (std::strcmp(p.group, groupName) != 0) continue;
    sliderRow(p, y);
    y += kRowH;
  }

  // Actions.
  const float by = kY + kH - 36;
  if (button("Save", kX + 14, by, 90)) {
    setStatus(saveTuning(config_.file, config_.params, config_.fileHeader)
                  ? TextFormat("Saved %s/%s", GetWorkingDirectory(), config_.file)
                  : "Save failed");
  }
  if (button("Load", kX + 112, by, 90)) {
    setStatus(loadTuning(config_.file, config_.params) ? TextFormat("Loaded %s", config_.file)
                                                       : TextFormat("No %s", config_.file));
  }
  if (button(TextFormat("Reset %s", groupName), kX + 210, by, 140)) {
    resetTuning(config_.params, groupName);
    setStatus("Tab reset to defaults");
  }
  if (button("Reset all", kX + 358, by, 100)) {
    resetTuning(config_.params);
    setStatus("All reset to defaults");
  }
  if (GetTime() - statusTime_ < 4.0) label(status_, kX + 14, by - 20, 12, kAmber);
}

} // namespace engine
