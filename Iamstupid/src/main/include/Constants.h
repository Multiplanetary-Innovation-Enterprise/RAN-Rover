// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <time.h>
#include <iostream>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

constexpr int kDriverControllerPort = 0;
constexpr int jDriverControllerPort = 1;
constexpr int iDriverControllerPort = 2;

}  // namespace OperatorConstants

namespace MADConstants {
    // CAN IDs
    constexpr int rearRightDriveID = 0;
    constexpr int frontRightDriveID = 0;
    constexpr int rearLeftDriveID = 0;
    constexpr int frontLeftDriveID = 0;

    // Autonomous speeds
    constexpr int leftSpeed = .2;
    constexpr int rightSpeed = .2;

    // constexpr int excavationZone = 0; //?? what was this for

}

namespace HADConstants {
    // CAN IDs
    constexpr int beltDriveID = 0;
    constexpr int depositBLID = 0;                  // "deposit bucket ladder ID"

    // DIO Ports
    constexpr int depositLinActID_dir = 0;
    constexpr int depositLinActID_vel = 0;
    // constexpr int depositLinActID_dir2 = 0;      // HAD does not know if they are using 1 or 2
    // constexpr int depositLinActID_vel2 = 0;      //  linear actuators yet

    // Motor Speeds
    constexpr double beltDriveSpeed = 0;
    constexpr double depositBLSpeed = 0;
    constexpr double depositLinAct_vel = 0;
    constexpr double depositLinAct_dir = 0;
    
}

namespace EXCConstants {
    // CAN IDs


    // DIO Ports


    // Motor Speeds
    constexpr double bucketLadderSpeed = 0;


    // Excavator Extension Positions
    constexpr int maxRetraction = 0;
    constexpr int maxExtension = 0;
    constexpr double linActuatorMax = 0;
    constexpr double linActuatorMin = 0;

}