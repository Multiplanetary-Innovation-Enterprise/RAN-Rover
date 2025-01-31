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

#include "subsystems/Mobility.h"
#include "subsystems/Hopper.h"
#include "subsystems/Excavation.h"
#include "subsystems/Deposition.h"
#include "sendables/IMUSendable.h"
#include "Vision.h"
#include "Autonomy.h"
#include "Teleop.h"

class Robot : public frc::TimedRobot {
 public:
  IMUSendable imu{frc::SPI::Port::kMXP};
 
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
  MobilitySubsystem mob;
  ExcavationSubsystem exc;
  HopperSubsystem hop;

  Teleop teleop{this, &mob, &exc, &hop};
  Autonomy autonomy{this, &vision, &mob, &exc, &hop};
};
