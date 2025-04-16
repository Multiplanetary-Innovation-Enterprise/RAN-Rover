#include "subsystems/Blinkin.h"

void Blinkin::Off() {
    blinkin.Set(BlinkinPattern::LIGHT_CHASE_RED);
}

void Blinkin::Set(double pattern) {
    blinkin.Set(pattern);
}