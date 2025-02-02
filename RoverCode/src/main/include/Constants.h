// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

namespace PortConstants {

    inline constexpr int primaryController = 0;
    inline constexpr int secondaryController = 1;

    inline constexpr int navX = 0;

    // Mobility
    inline constexpr int mobMotor[4] =  {11, 13,
                                         10, 12};
    inline constexpr int mobActVel[4] = {-1, -1,
                                         -1, -1};
    inline constexpr int mobActDir[4] = {-1, -1,
                                         -1, -1};
    inline constexpr int mobActPot[4] = {-1, -1,
                                         -1, -1};

    // Excavation
    inline constexpr int excSpin = 14;
    inline constexpr int excLeftActVel = 0; // Digital Output
    inline constexpr int excLeftActDir = 1; // Digital Output
    inline constexpr int excLeftPot = 0; // Analog Input

    inline constexpr int excRightActVel = 2; // Digital Output
    inline constexpr int excRightActDir = 3; // Digital Output
    inline constexpr int excRightPot = 1; // Analog Input

    // Hopper
    inline constexpr int hopSpin = 15;

    // Deposition
    inline constexpr int depSpin = 16;
    
    inline constexpr int depLeftActVel = 4; // Digital Output
    inline constexpr int depLeftActDir = 5; // Digital Output
    inline constexpr int depLeftPot = 2; // Analog Input

    inline constexpr int depRightActVel = 6; // Digital Output
    inline constexpr int depRightActDir = 7; // Digital Output
    inline constexpr int depRightPot = 3; // Analog Input
}

namespace GearRatioConstants {

    inline constexpr int hop = 80; // 80 : 1
}