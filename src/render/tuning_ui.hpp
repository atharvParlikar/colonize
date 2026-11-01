#pragma once

#include "player/player.hpp"

// In-game flight tuning panel (Tab). While it is open the cursor is free and
// the mouse no longer flies the craft; the sim keeps running, so the keys
// still fly and you can see each change take effect.
//   click / drag a slider    set the value
//   Shift + drag              fine adjust (1/10)
//   mouse wheel on a slider   nudge (Shift: finer)
//   right click a slider      back to its default
// Save writes tuning.txt in the working directory; it loads at startup.
constexpr const char *kTuningFile = "tuning.txt";

bool tuningPanelOpen();
void toggleTuningPanel();
// Draws the panel and handles its mouse input. Call after the HUD.
void drawTuningPanel(const Player &player);
