// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

namespace PortConstants {

// Input
    inline constexpr int primaryController = 0;
    inline constexpr int secondaryController = 1;

// Output

    // Mobility
    inline constexpr int mobBackLeft = 10;
    inline constexpr int mobFrontLeft = 11;
    inline constexpr int mobBackRight = 12;
    inline constexpr int mobFrontRight = 13;
    
    // Excavation
    inline constexpr int excSpin = 14;
    inline constexpr int excLeftActVel = 0; // DIO
    inline constexpr int excLeftActDir = 1; // DIO
    inline constexpr int excRightActVel = 2; // DIO
    inline constexpr int excRightActDir = 3; // DIO

    // Hopper
    inline constexpr int hopSpin = 15;

    // Deposition
    inline constexpr int depSpin = 16;
}

namespace AutoConstants {

    const std::string Default = "Default";
    const std::string DriveCycle = "DriveCycle";
}