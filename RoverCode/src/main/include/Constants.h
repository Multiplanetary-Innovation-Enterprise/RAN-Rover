// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "sendables/Coord.h"

namespace PortConstants {

    // Controllers
    const int primaryController = 0;
    const int secondaryController = 1;

    // Mobility
    const int mobMotor[4] =  {1, 2,
                              3, 4};
    const int mobActVel[4] = {0, 4,
                              2, 6}; // Digital Output (On RoboRio)
    const int mobActDir[4] = {1, 5,
                              3, 7}; // Digital Output (On RoboRio)
    const int mobActPot[4] = {0, 2,
                              1, 3}; // Analog Input (On RoboRio)

    // Excavation
    const int excSpin = 5;
    const int excLeftActVel = 10; // Digital Output (0 On NavX MXP Board)
    const int excLeftActDir = 11; // Digital Output (1 On NavX MXP Board)
    const int excLeftPot = 4; // Analog Input (2 On NavX MXP Board)
    
    const int excRightActVel = 12; // Digital Output (2 On NavX MXP Board)
    const int excRightActDir = 13; // Digital Output (3 On NavX MXP Board)
    const int excRightPot = 5; // Analog Input (3 On NavX MXP Board)

    // Hopper
    const int hopSpin = 6;

    // REV Blinkin LED Lights
    const int blinkin = 0;
}

namespace KSC_ArenaConstants {
    
    // Following the NASA Guidebook
    // All units are in meters
    // For pairs of values it is always listed x, y.
    // 0, 0 is assumed to be the corner of the starting zone.

    const Coord size{6.88, 5.0};

    const Coord traversalZonePos{0.0, 0.0};
    const Coord traversalZoneSize{3.88, 5.0};

    const Coord excavationZonePos{3.88, 2.0};
    const Coord excavationZoneSize{3.0, 3.0};

    const Coord constructionZonePos{3.88, 0.0};
    const Coord constructionZoneSize{3.0, 2.0};

    const Coord bermCenter{5.38, 0.6};
    const Coord bermSize{2.0, 0.7};
}

namespace UCF_ArenaConstants {

    // Following the University of Central Forida's Guidebook
    // All units are in meters
    // For pairs of values it is always listed x, y.
    // 0, 0 is assumed to be the corner of the starting zone.

    const Coord size{4.57, 8.14};

    const Coord traversalZonePos{0.0, 0.0};
    const Coord traversalZoneSize{4.57, 4.07};

    const Coord excavationZonePos{0.0, 4.07};
    const Coord excavationZoneSize{4.57, 4.07};

    const Coord constructionZonePos{2.57, 5.54};
    const Coord constructionZoneSize{2.0, 2.6};

    // Berm position is estimated to be the center of construction zone as the UCF Guidebook does not specify.
    const Coord bermCenter{3.57, 6.84};
    const Coord bermSize{0.7, 2.0};
}