#pragma once

#include <frc/motorcontrol/Spark.h>
#include "Constants.h"

class BlinkinPattern {
    public:
        // Fixed Palette Pattern
        inline static const double RAINBOW_RAINBOW_PALETTE = -0.99;
        inline static const double RAINBOW_PARTY_PALETTE = -0.97;
        inline static const double RAINBOW_OCEAN_PALETTE = -0.95;
        inline static const double RAINBOW_LAVA_PALETTE = -0.93;
        inline static const double RAINBOW_FOREST_PALETTE = -0.91;
        inline static const double RAINBOW_WITH_GLITTER = -0.89;
        inline static const double CONFETTI = -0.87;
        inline static const double SHOT_RED = -0.85;
        inline static const double SHOT_BLUE = -0.83;
        inline static const double SHOT_WHITE = -0.81;
        inline static const double SINELON_RAINBOW_PALETTE = -0.79;
        inline static const double SINELON_PARTY_PALETTE = -0.77;
        inline static const double SINELON_OCEAN_PALETTE = -0.75;
        inline static const double SINELON_LAVA_PALETTE = -0.73;
        inline static const double SINELON_FOREST_PALETTE = -0.71;
        inline static const double BEATS_PER_MINUTE_RAINBOW_PALETTE = -0.69;
        inline static const double BEATS_PER_MINUTE_PARTY_PALETTE = -0.67;
        inline static const double BEATS_PER_MINUTE_OCEAN_PALETTE = -0.65;
        inline static const double BEATS_PER_MINUTE_LAVA_PALETTE = -0.63;
        inline static const double BEATS_PER_MINUTE_FOREST_PALETTE = -0.61;
        inline static const double FIRE_MEDIUM = -0.59;
        inline static const double FIRE_LARGE = -0.57;
        inline static const double TWINKLES_RAINBOW_PALETTE = -0.55;
        inline static const double TWINKLES_PARTY_PALETTE = -0.53;
        inline static const double TWINKLES_OCEAN_PALETTE = -0.51;
        inline static const double TWINKLES_LAVA_PALETTE = -0.49;
        inline static const double TWINKLES_FOREST_PALETTE = -0.47;
        inline static const double COLOR_WAVES_RAINBOW_PALETTE = -0.45;
        inline static const double COLOR_WAVES_PARTY_PALETTE = -0.43;
        inline static const double COLOR_WAVES_OCEAN_PALETTE = -0.41;
        inline static const double COLOR_WAVES_LAVA_PALETTE = -0.39;
        inline static const double COLOR_WAVES_FOREST_PALETTE = -0.37;
        inline static const double LARSON_SCANNER_RED = -0.35;
        inline static const double LARSON_SCANNER_GRAY = -0.33;
        inline static const double LIGHT_CHASE_RED = -0.31;
        inline static const double LIGHT_CHASE_BLUE = -0.29;
        inline static const double LIGHT_CHASE_GRAY = -0.27;
        inline static const double HEARTBEAT_RED = -0.25;
        inline static const double HEARTBEAT_BLUE = -0.23;
        inline static const double HEARTBEAT_WHITE = -0.21;
        inline static const double HEARTBEAT_GRAY = -0.19;
        inline static const double BREATH_RED = -0.17;
        inline static const double BREATH_BLUE = -0.15;
        inline static const double BREATH_GRAY = -0.13;
        inline static const double STROBE_RED = -0.11;
        inline static const double STROBE_BLUE = -0.09;
        inline static const double STROBE_GOLD = -0.07;
        inline static const double STROBE_WHITE = -0.05;

        // CP1: Color 1 Pattern
        inline static const double CP1_END_TO_END_BLEND_TO_BLACK = -0.03;
        inline static const double CP1_LARSON_SCANNER = -0.01;
        inline static const double CP1_LIGHT_CHASE = +0.01;
        inline static const double CP1_HEARTBEAT_SLOW = +0.03;
        inline static const double CP1_HEARTBEAT_MEDIUM = +0.05;
        inline static const double CP1_HEARTBEAT_FAST = +0.07;
        inline static const double CP1_BREATH_SLOW = +0.09;
        inline static const double CP1_BREATH_FAST = +0.11;
        inline static const double CP1_SHOT = +0.13;
        inline static const double CP1_STROBE = +0.15;

        // CP2: Color 2 Pattern
        inline static const double CP2_END_TO_END_BLEND_TO_BLACK = +0.17;
        inline static const double CP2_LARSON_SCANNER = +0.19;
        inline static const double CP2_LIGHT_CHASE = +0.21;
        inline static const double CP2_HEARTBEAT_SLOW = +0.23;
        inline static const double CP2_HEARTBEAT_MEDIUM = +0.25;
        inline static const double CP2_HEARTBEAT_FAST = +0.27;
        inline static const double CP2_BREATH_SLOW = +0.29;
        inline static const double CP2_BREATH_FAST = +0.31;
        inline static const double CP2_SHOT = +0.33;
        inline static const double CP2_STROBE = +0.35;
        
        // CP1_2: Color 1 and 2 Pattern
        inline static const double CP1_2_SPARKLE_1_ON_2 = +0.37;
        inline static const double CP1_2_SPARKLE_2_ON_1 = +0.39;
        inline static const double CP1_2_COLOR_GRADIENT = +0.41;
        inline static const double CP1_2_BEATS_PER_MINUTE = +0.43;
        inline static const double CP1_2_END_TO_END_BLEND_1_TO_2 = +0.45;
        inline static const double CP1_2_END_TO_END_BLEND = +0.47;
        inline static const double CP1_2_NO_BLENDING = +0.49;
        inline static const double CP1_2_TWINKLES = +0.51;
        inline static const double CP1_2_COLOR_WAVES = +0.53;
        inline static const double CP1_2_SINELON = +0.55;

        // Solid color
        inline static const double HOT_PINK = +0.57;
        inline static const double DARK_RED = +0.59;
        inline static const double RED = +0.61;
        inline static const double RED_ORANGE = +0.63;
        inline static const double ORANGE = +0.65;
        inline static const double GOLD = +0.67;
        inline static const double YELLOW = +0.69;
        inline static const double LAWN_GREEN = +0.71;
        inline static const double LIME = +0.73;
        inline static const double DARK_GREEN = +0.75;
        inline static const double GREEN = +0.77;
        inline static const double BLUE_GREEN = +0.79;
        inline static const double AQUA = +0.81;
        inline static const double SKY_BLUE = +0.83;
        inline static const double DARK_BLUE = +0.85;
        inline static const double BLUE = +0.87;
        inline static const double BLUE_VIOLET = +0.89;
        inline static const double VIOLET = +0.91;
        inline static const double WHITE = +0.93;
        inline static const double GRAY = +0.95;
        inline static const double DARK_GRAY = +0.97;
        inline static const double BLACK = +0.99;
};

class Blinkin {
    public:
        static void Off();
        static void Set(double pattern);
    private:
        inline static frc::Spark blinkin{PortConstants::blinkin};
};