#pragma once

#include "player/player.hpp"

// Fully synthesised audio (no sample files): jet engine and wind, generated
// on the audio thread from a few parameters published once per frame.
void initAudio();
void updateAudio(const Player &player);
void shutdownAudio();
