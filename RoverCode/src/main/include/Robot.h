// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#define _USE_MATH_DEFINES

#include <string>
#include <cmath>

#include <frc2/command/Commands.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <wpi/raw_ostream.h>
#include <frc/SPI.h>

#include "Constants.h"
#include "subsystems/Mobility.h"
#include "subsystems/Hopper.h"
#include "subsystems/Excavation.h"
#include "subsystems/Deposition.h"
#include "sendables/IMUSendable.h"

class Robot : public frc::TimedRobot {
 public:
  IMUSendable imu{frc::SPI::Port::kMXP};
  frc::XboxController controller{PortConstants::controller};

  std::string autoSelected;
  std::string schemeSelected;

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
  frc::SendableChooser<std::string> autoChooser;
  frc::SendableChooser<std::string> schemeChooser;
  double leftStickDeadzone = 0.1;
  double rightStickDeadzone = 0.1;

  MobilitySubsystem mob;
  HopperSubsystem hop;
  ExcavationSubsystem exc;
  DepositionSubsystem dep;
};
