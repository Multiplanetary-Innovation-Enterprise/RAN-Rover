// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#define _USE_MATH_DEFINES

#include <string>
#include <cmath>

#include <frc2/command/Commands.h>
#include <frc/TimedRobot.h>
#include <wpi/raw_ostream.h>

#include "Autonomy.h"
#include "Teleop.h"
#include "Vision.h"
#include "subsystems/Mobility.h"
#include "subsystems/Hopper.h"
#include "subsystems/Excavation.h"
#include "subsystems/Blinkin.h"
#include "sendables/IMUSendable.h"

class Robot : public frc::TimedRobot {
 public:
  IMUSendable imu{frc::SerialPort::Port::kUSB2};

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

  void Kill();
 private:
  Vision vision;
  Localization* localization;
  MobilitySubsystem mob;
  ExcavationSubsystem exc;
  HopperSubsystem hop;

  frc::Timer pathingTimer;

  Teleop teleop;
  Autonomy autonomy;
};
