// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::DashboardInit() {
  frc::SmartDashboard::PutData("Auto Modes", &chooser);

  frc::ShuffleboardTab& inputConfigTab = frc::Shuffleboard::GetTab("Input Configuration");
  frc::ShuffleboardLayout& primaryDeadzones = inputConfigTab.GetLayout("Primary Controller Deadzones", frc::BuiltInLayouts::kGrid)
    .WithSize(6, 4)
    .WithPosition(0, 0)
    .WithProperties({
      {"Label position", nt::Value::MakeString("TOP")}
    });
  primaryDeadzones.AddPersistent("Left Trigger", pLeftTriggerDeadzone)
          .WithWidget(frc::BuiltInWidgets::kNumberSlider)
          .WithPosition(0, 0)
          .WithProperties({
            {"min", nt::Value::MakeDouble(0)},
            {"max", nt::Value::MakeDouble(1)}
          });
  primaryDeadzones.AddPersistent("Right Trigger", pRightTriggerDeadzone)
          .WithWidget(frc::BuiltInWidgets::kNumberSlider)
          .WithPosition(1, 0)
          .WithProperties({
            {"min", nt::Value::MakeDouble(0)},
            {"max", nt::Value::MakeDouble(1)}
          });
  primaryDeadzones.AddPersistent("Left Stick", pLeftStickDeadzone)
          .WithWidget(frc::BuiltInWidgets::kNumberSlider)
          .WithPosition(0, 1)
          .WithProperties({
            {"min", nt::Value::MakeDouble(0)},
            {"max", nt::Value::MakeDouble(1)}
          });
  primaryDeadzones.AddPersistent("Right Stick", pRightStickDeadzone)
          .WithWidget(frc::BuiltInWidgets::kNumberSlider)
          .WithPosition(1, 1)
          .WithProperties({
            {"min", nt::Value::MakeDouble(0)},
            {"max", nt::Value::MakeDouble(1)}
          });
  frc::ShuffleboardLayout& secondaryDeadzones = inputConfigTab.GetLayout("Secondary Controller Deadzones", frc::BuiltInLayouts::kGrid)
    .WithSize(6, 4)
    .WithPosition(6, 0)
    .WithProperties({
      {"Label position", nt::Value::MakeString("TOP")}
    });
  secondaryDeadzones.AddPersistent("Left Trigger", sLeftTriggerDeadzone)
          .WithWidget(frc::BuiltInWidgets::kNumberSlider)
          .WithPosition(0, 0)
          .WithProperties({
            {"min", nt::Value::MakeDouble(0)},
            {"max", nt::Value::MakeDouble(1)}
          });
  secondaryDeadzones.AddPersistent("Right Trigger", sRightTriggerDeadzone)
          .WithWidget(frc::BuiltInWidgets::kNumberSlider)
          .WithPosition(1, 0)
          .WithProperties({
            {"min", nt::Value::MakeDouble(0)},
            {"max", nt::Value::MakeDouble(1)}
          });
  secondaryDeadzones.AddPersistent("Left Stick", sLeftStickDeadzone)
          .WithWidget(frc::BuiltInWidgets::kNumberSlider)
          .WithPosition(0, 1)
          .WithProperties({
            {"min", nt::Value::MakeDouble(0)},
            {"max", nt::Value::MakeDouble(1)}
          });
  secondaryDeadzones.AddPersistent("Right Stick", sRightStickDeadzone)
          .WithWidget(frc::BuiltInWidgets::kNumberSlider)
          .WithPosition(1, 1)
          .WithProperties({
            {"min", nt::Value::MakeDouble(0)},
            {"max", nt::Value::MakeDouble(1)}
          });

  frc::ShuffleboardTab& roverConfigTab = frc::Shuffleboard::GetTab("Rover Configuration");

  frc::ShuffleboardLayout& mobList = roverConfigTab.GetLayout("Mobility", frc::BuiltInLayouts::kList)
      .WithSize(8, 4)
      .WithPosition(0, 0)
      .WithProperties({
      {"Label position", nt::Value::MakeString("LEFT")}
      });
  mobList.Add("Max Drive Speed", mob.maxDriveSpeed) 
          .WithWidget(frc::BuiltInWidgets::kNumberSlider)
          .WithProperties({
              {"min", nt::Value::MakeDouble(0)},
              {"max", nt::Value::MakeDouble(1)}
          });
  mobList.Add("Max Crawl Speed", mob.maxCrawlSpeed)
          .WithWidget(frc::BuiltInWidgets::kNumberSlider)
          .WithProperties({
              {"min", nt::Value::MakeDouble(0)},
              {"max", nt::Value::MakeDouble(1)}
          });
  mobList.Add("Slip Current Threshold", mob.currentThreshold)
          .WithWidget(frc::BuiltInWidgets::kField);
  mobList.Add("Slip Off Cycles", mob.cyclesSlipOff)
          .WithWidget(frc::BuiltInWidgets::kField);
  mobList.Add("Slip Run Cycles", mob.cyclesSlipRun)
          .WithWidget(frc::BuiltInWidgets::kField);

  frc::ShuffleboardLayout& excList = roverConfigTab.GetLayout("Excavation", frc::BuiltInLayouts::kList)
      .WithSize(8, 2)
      .WithPosition(8, 0)
      .WithProperties({
      {"Label position", nt::Value::MakeString("LEFT")}
      });
  excList.Add("Max Spin Speed", exc.maxSpinSpeed)
          .WithWidget(frc::BuiltInWidgets::kNumberSlider)
          .WithProperties({
              {"min", nt::Value::MakeDouble(0)},
              {"max", nt::Value::MakeDouble(1)}
          });

  frc::ShuffleboardLayout& depList = roverConfigTab.GetLayout("Deposition", frc::BuiltInLayouts::kList)
      .WithSize(8, 2)
      .WithPosition(8, 4)
      .WithProperties({
      {"Label position", nt::Value::MakeString("LEFT")}
      });;
  depList.Add("Max Spin Speed", dep.maxSpinSpeed)
          .WithWidget(frc::BuiltInWidgets::kNumberSlider)
          .WithProperties({
              {"min", nt::Value::MakeDouble(0)},
              {"max", nt::Value::MakeDouble(1)}
          });

  frc::ShuffleboardLayout& hopList = roverConfigTab.GetLayout("Hopper", frc::BuiltInLayouts::kList)
      .WithSize(8, 2)
      .WithPosition(8, 2)
      .WithProperties({
      {"Label position", nt::Value::MakeString("LEFT")}
      });;
  hopList.Add("Max Spin Speed", hop.maxSpinSpeed)
          .WithWidget(frc::BuiltInWidgets::kNumberSlider)
          .WithProperties({
              {"min", nt::Value::MakeDouble(0)},
              {"max", nt::Value::MakeDouble(1)}
          });
}

void Robot::RobotInit() {
  chooser.SetDefaultOption(AutoConstants::Default, AutoConstants::Default);
  chooser.AddOption(AutoConstants::DriveCycle, AutoConstants::DriveCycle);

  DashboardInit();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  // Stops all subsystem motors, and resets all of their variables to their initial states.
  mob.Reset();
  hop.Reset();
  exc.Reset();
  dep.Reset();
  
  autoSelected = chooser.GetSelected();
  fmt::print("Auto selected: {}\n", autoSelected);

  if (autoSelected == AutoConstants::DriveCycle) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (autoSelected == AutoConstants::DriveCycle) {
    // Custom Auto goes here
    wpi::outs() << "Auto Drive Cycle\n";
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
  // Stops all subsystem motors, and resets all of their variables to their initial states.
  mob.Reset();
  hop.Reset();
  exc.Reset();
  dep.Reset();
}

void Robot::TeleopPeriodic() {

/****************
 *   MOBILITY   *
 ****************/

  // Left Joystick Y-Axis on Primary Controller
  double leftStick = Utils::Deadzone(primaryController.GetLeftY(), pLeftStickDeadzone);
  // Right Joystick Y-Axis on Primary Controller
  double rightStick = Utils::Deadzone(primaryController.GetRightY(), pRightStickDeadzone);

  // Either of the Joystick Buttons have been pressed on Primary Controller
  bool sticksClicked = primaryController.GetLeftStickButton() || primaryController.GetRightStickButton();

  // If there is a stick input, drive, otherwise, stop driving.
  if (leftStick > 0.0 || rightStick > 0.0) {
    // Mobility Drive takes the left side input, right side input, and a boolean to determine if it should crawl
    mob.Drive(leftStick, rightStick, sticksClicked);
  } else {
    mob.Stop();
  }


/****************
 *    HOPPER    *
 ****************/

  // If either of the bumpers are pressed, spin the hopper.
  if (primaryController.GetLeftBumper() || primaryController.GetRightBumper()) {
    // Hopper Spin takes a boolean to determine the direction to spin.
    hop.Spin(1.0, primaryController.GetRightBumper());
  } else {
    hop.Stop();
  }


/****************
 *  EXCAVATION  *
 ****************/

  // Right Trigger on Primary Controller
  double rightTrigger = Utils::Deadzone(primaryController.GetRightTriggerAxis(), pRightTriggerDeadzone);

  // If there is an input from the right trigger, spin the excavator.
  if (rightTrigger > 0.0) {
    // Excavation Spin takes a boolean to determine the direction to spin.
    exc.Spin(rightTrigger, primaryController.GetStartButton());
  } else {
    exc.Stop();
  }


/****************
 *  DEPOSITION  *
 ****************/

  // Left Trigger on Primary Controller
  double leftTrigger = Utils::Deadzone(primaryController.GetLeftTriggerAxis(), pLeftTriggerDeadzone);

  // If there is an input from the left trigger, spin the depositor.
  if (leftTrigger > 0.0) {
    // Deposition Spin takes a boolean to determine the direction to spin.
    dep.Spin(leftTrigger, primaryController.GetBackButton());
  } else {
    dep.Stop();
  }
}

void Robot::DisabledInit() {
  // Stops all subsystem motors, and resets all of their variables to their initial states.
  mob.Reset();
  hop.Reset();
  exc.Reset();
  dep.Reset();
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
  // Stops all subsystem motors, and resets all of their variables to their initial states.
  mob.Reset();
  hop.Reset();
  exc.Reset();
  dep.Reset();
}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {
  // Stops all subsystem motors, and resets all of their variables to their initial states.
  mob.Reset();
  hop.Reset();
  exc.Reset();
  dep.Reset();
}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
