// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc2/command/button/Trigger.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {

  std::vector<int> defaultAutoPhases = {0, 1, 1, 1, 1};
  frc::SmartDashboard::PutBooleanArray("Auto Phases", defaultAutoPhases);

  schemeChooser.SetDefaultOption(ControlSchemes::Atlas, ControlSchemes::Atlas);
  schemeChooser.AddOption(ControlSchemes::Stellar, ControlSchemes::Stellar);
  frc::SmartDashboard::PutData("Control Scheme", &schemeChooser);

  frc::SmartDashboard::PutNumber("Left Stick Deadzone", leftStickDeadzone);
  frc::SmartDashboard::SetPersistent("Left Stick Deadzone");
  frc::SmartDashboard::PutNumber("Right Stick Deadzone", rightStickDeadzone);
  frc::SmartDashboard::SetPersistent("Right Stick Deadzone");
  
  frc::SmartDashboard::PutData("IMU", &imu);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();
}

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
  angleAct.Cancel();
  distDrive.Cancel();
  
  autoSelected = frc::SmartDashboard::GetBooleanArray("Auto Phases");
  fmt::print("Auto selected: {}\n", autoSelected);

  if (autoSelected[AutoConstants::Traversal] != 0) {
    // Find EXC Beacon
        
    // If angle non-zero
    // Steer to angle
    angleAct.Set(0, vision.getTagAngle(0));
    angleAct.Schedule();

    // If distance is > threshold
    // Drive
    distDrive.Set(vision.getTagDistance(0));
    distDrive.Schedule();

    // Stop Drive
  }

  if (autoSelected[AutoConstants::ExcOrientation] != 0) {


  }

  if (autoSelected[AutoConstants::ExcAction] != 0) {


  }

  if (autoSelected[AutoConstants::DepOrientation] != 0) {


  }

  if (autoSelected[AutoConstants::DepAction] != 0) {


  }
}

void Robot::AutonomousPeriodic() {

  if (autoSelected[AutoConstants::Traversal] != 0) {
    // Find EXC Beacon
    
    // If angle non-zero
    // Steer to angle
    angleAct.Set(0, vision.getTagAngle(0));
    
    distDrive.Set(vision.getTagDistance(0));
    if (!distDrive.IsFinished())
      angleAct.Schedule();
    // Stop Drive
  }

  if (autoSelected[AutoConstants::ExcOrientation] != 0) {


  }

  if (autoSelected[AutoConstants::ExcAction] != 0) {


  }

  if (autoSelected[AutoConstants::DepOrientation] != 0) {


  }

  if (autoSelected[AutoConstants::DepAction] != 0) {
    // Assume we are aligned to construction berm, that should have been done in DEP Orientation

    // TODO: Will need to be changed to dep when we switch to Atlas
    // Spin Deposition Hopper so the belt has done one full rotation.
    hop.Spin(0.75, false);
    sleep(6); // Number will need to be adjusted to represent one full rotation at the given motor speed.
    hop.Stop(); 
  }
}

void Robot::TeleopInit() {
  // Stops all subsystem motors, and resets all of their variables to their initial states.
  mob.Reset();
  hop.Reset();
  exc.Reset();
  dep.Reset();
  angleAct.Cancel();
  distDrive.Cancel();

  schemeSelected = schemeChooser.GetSelected();
  fmt::print("Scheme selected: {}\n", schemeSelected);
}

void Robot::TeleopPeriodic() {

  leftStickDeadzone = frc::SmartDashboard::GetNumber("Left Stick Deadzone", leftStickDeadzone);
  rightStickDeadzone = frc::SmartDashboard::GetNumber("Right Stick Deadzone", rightStickDeadzone);

  if (schemeSelected == ControlSchemes::Stellar) { 
#pragma region Stellar

    /****************
     *   MOBILITY   *
     ****************/
  #pragma region Stellar_Mobility
    // Left Joystick Y-Axis on Primary Controller
    double leftStick = controller.GetLeftY();
    // Right Joystick Y-Axis on Primary Controller
    double rightStick = controller.GetRightY();

    // Either of the Joystick Buttons have been pressed on Primary Controller
    bool sticksClicked = controller.GetLeftStickButton() || controller.GetRightStickButton();

    // If there is a stick input, drive, otherwise, stop driving.
    if (abs(leftStick) > leftStickDeadzone || abs(rightStick) > rightStickDeadzone) {
      // Mobility Drive takes the left side input, right side input, and a boolean to determine if it should crawl
      mob.Drive(-leftStick, -rightStick, sticksClicked);
    } else {
      mob.Stop();
    }
  #pragma endregion Stellar_Mobility

    /****************
     *    HOPPER    *
     ****************/
  #pragma region Stellar_Hopper
    // If either of the bumpers are pressed, spin the hopper.
    if (controller.GetLeftBumper() || controller.GetRightBumper()) {
      // Hopper Spin takes a boolean to determine the direction to spin.
      hop.Spin(1.0, controller.GetRightBumper());
    } else {
      hop.Stop();
    }
  #pragma endregion Stellar_Hopper

    /****************
     *  EXCAVATION  *
     ****************/
  #pragma region Stellar_Excavation
    // Right Trigger on Primary Controller
    double rightTrigger = controller.GetRightTriggerAxis();

    // If there is an input from the right trigger, spin the excavator.
    if (rightTrigger > 0.0) {
      // Excavation Spin takes a boolean to determine the direction to spin.
      exc.Spin(rightTrigger, controller.GetStartButton());
    } else {
      exc.Stop();
    }

    if (!controller.GetYButton() != !controller.GetAButton()) { // XOR, only actuate when one is pressed.
      exc.StartActuate(controller.GetAButton());
    } else {
      exc.StopActuate(false);
    }
  #pragma endregion Stellar_Excavation

    /****************
     *  DEPOSITION  *
     ****************/
  #pragma region Stellar_Deposition
    // Left Trigger on Primary Controller
    double leftTrigger = controller.GetLeftTriggerAxis();

    // If there is an input from the left trigger, spin the depositor.
    if (leftTrigger > 0.0) {
      // Deposition Spin takes a boolean to determine the direction to spin.
      dep.Spin(leftTrigger, controller.GetBackButton());
    } else {
      dep.Stop();
    }

    if (controller.GetPOV() == 0 || controller.GetPOV() == 180) { // XOR, only actuate when one is pressed.
      dep.StartActuate(controller.GetPOV() == 180);
    } else {
      dep.StopActuate(false);
    }
  #pragma endregion Stellar_Deposition
    
#pragma endregion Stellar
  } else if (schemeSelected == ControlSchemes::Atlas) {
#pragma region Atlas
    /****************
     *   MOBILITY   *
     ****************/
  #pragma region Atlas_Mobility

    // Left Joystick X on Primary Controller
    double leftStickX = controller.GetLeftX();
    // Left Joystick Y on Primary Controller
    double leftStickY = -controller.GetLeftY();
    // Left Joystick Unnormalized Magnitude on Primary Controller
    double leftStickMagnitude = std::min(sqrt(leftStickX * leftStickX + leftStickY * leftStickY), 1.0);
    // Left Joystick Angle on Primary Controller
    double leftStickAngle = int(360 + atan2(leftStickX, leftStickY) * 180.0 / M_PI) % 360; // In degrees, 0 is forward, 90 is right, 180 is backward, 270 is left

    // Right Joystick X and Y on Primary Controller
    double rightStickX = controller.GetRightX();
    double rightStickY = -controller.GetRightY();
    // Right Joystick Angle on Primary Controller
    double rightStickAngle = int(360 + atan2(rightStickX, rightStickY) * 180.0 / M_PI) % 360; // In degrees, 0 is forward, 90 is right, 180 is backward, 270 is left

    // D-Pad on Controller for Crawl (Slow drive)
    if (controller.GetPOV() == 0) { // D-Pad Up
      mob.Drive(1, 1, true);
    } else if (controller.GetPOV() == 180) { // D-Pad Down
      mob.Drive(-1, -1, true);
    } else {
      // If not crawling, allow for normal stick input.

      // If there is a stick input, drive, otherwise, stop driving.
      if (leftStickMagnitude > leftStickDeadzone) {

        // Stick zone for negation of magnitude
        if (leftStickAngle > 120 && leftStickAngle < 240)
          leftStickMagnitude *= -1;

        wpi::outs() << "Steer Angle: " << std::to_string(int(leftStickAngle)) << "\n";
        // Mobility Drive takes the left side input, right side input, and a boolean to determine if it should crawl
        mob.Drive(leftStickMagnitude, leftStickMagnitude, false);
      } else {
        mob.Stop();
      }
    }

  #pragma endregion Atlas_Mobility

    /****************
     *    HOPPER    *
     ****************/
  #pragma region Atlas_Hopper
    // Left Trigger on Controller
    double leftTrigger = controller.GetLeftTriggerAxis();

    // If there is an input from the left trigger, spin the Hopper.
    if (leftTrigger > 0.0) {
      // Hopper Spin takes a boolean to determine the direction to spin.
      hop.Spin(leftTrigger, controller.GetLeftBumper());
    } else {
      hop.Stop();
    }
    
    // Holding X Button Locks the current speed and direction of the Hopper.
    if (controller.GetXButtonPressed()) {
      hop.HoldLock(true);
    } else if (controller.GetXButtonReleased()) {
      hop.HoldLock(false);
    }
  #pragma endregion Atlas_Hopper

    /****************
     *  EXCAVATION  *
     ****************/
  #pragma region Atlas_Excavation
    // Right Trigger on Controller
    double rightTrigger = controller.GetRightTriggerAxis();

    // If there is an input from the right trigger, spin the Excavator.
    if (rightTrigger > 0.0) {
      // Excavation Spin takes a boolean to determine the direction to spin.
      exc.Spin(rightTrigger, controller.GetRightBumper());
    } else {
      exc.Stop();
    }
    
    // Holding B Button Locks the current speed and direction of the Excavator.
    if (controller.GetBButtonPressed()) {
      exc.HoldLock(true);
    } else if (controller.GetBButtonReleased()) {
      exc.HoldLock(false);
    }

    if (!controller.GetYButton() != !controller.GetAButton()) { // XOR, only actuate when one is pressed.
      exc.StartActuate(controller.GetAButton());
    } else {
      exc.StopActuate(false);
    }
  #pragma endregion Atlas_Excavation

#pragma endregion Atlas
  }
}

void Robot::DisabledInit() {
  // Stops all subsystem motors, and resets all of their variables to their initial states.
  mob.Reset();
  hop.Reset();
  exc.Reset();
  dep.Reset();
  angleAct.Cancel();
  distDrive.Cancel();
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
  // Stops all subsystem motors, and resets all of their variables to their initial states.
  mob.Reset();
  hop.Reset();
  exc.Reset();
  dep.Reset();
  angleAct.Cancel();
  distDrive.Cancel();
}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {
  // Stops all subsystem motors, and resets all of their variables to their initial states.
  mob.Reset();
  hop.Reset();
  exc.Reset();
  dep.Reset();
  angleAct.Cancel();
  distDrive.Cancel();
}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
