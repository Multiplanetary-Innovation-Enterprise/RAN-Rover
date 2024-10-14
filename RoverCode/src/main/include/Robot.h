// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc2/command/Commands.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <wpi/raw_ostream.h>

#include "Utils.h"
#include "Constants.h"
#include "subsystems/Mobility.h"
#include "subsystems/Hopper.h"
#include "subsystems/Excavation.h"
#include "subsystems/Deposition.h"

class Robot : public frc::TimedRobot {
 public:
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
  
 private:
  void DashboardInit();

  frc::SendableChooser<std::string> chooser;
  std::string autoSelected;

  MobilitySubsystem mob;
  HopperSubsystem hop;
  ExcavationSubsystem exc;
  DepositionSubsystem dep;

  frc::XboxController primaryController{PortConstants::primaryController};
  double pLeftStickDeadzone = 0.1;
  double pRightStickDeadzone = 0.1;
  double pLeftTriggerDeadzone = 0.1;
  double pRightTriggerDeadzone = 0.1;

  frc::XboxController secondaryController{PortConstants::secondaryController};
  double sLeftStickDeadzone = 0.1;
  double sRightStickDeadzone = 0.1;
  double sLeftTriggerDeadzone = 0.1;
  double sRightTriggerDeadzone = 0.1;
};
