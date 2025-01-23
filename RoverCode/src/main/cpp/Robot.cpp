// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  
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
  Kill();
  autonomy.Init();
}

void Robot::AutonomousPeriodic() { 
  autonomy.Periodic();
}

void Robot::TeleopInit() {
  // Stops all subsystem motors, and resets all of their variables to their initial states.
  Kill();
  teleop.Init();
}

void Robot::TeleopPeriodic() {
  teleop.Periodic();
}

void Robot::DisabledInit() {
  // Stops all subsystem motors, and resets all of their variables to their initial states.
  Kill();
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
  // Stops all subsystem motors, and resets all of their variables to their initial states.
  Kill();
}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {
  // Stops all subsystem motors, and resets all of their variables to their initial states.
  Kill();
}

void Robot::SimulationPeriodic() {}

void Robot::Kill() {
  wpi::outs() << "Kill Command Issued";
  frc2::CommandScheduler::GetInstance().CancelAll();
  mob.Reset();
  exc.Reset();
  hop.Reset();
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif