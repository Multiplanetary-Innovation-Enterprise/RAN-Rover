// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {

  frc::SmartDashboard::PutData("IMU", &imu);

  wpi::outs() << "Robot Init\n";
  localization = new Localization(&vision, &imu);
  teleop.SetSystems(&mob, &exc, &hop);  
  autonomy.SetSystems(&vision, localization, &mob, &exc, &hop);

  localization->Init();
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
  wpi::outs() << "Auto Init\n";
  Kill();
  autonomy.Init();
}

void Robot::AutonomousPeriodic() { 
  vision.IdentifyTags();
  localization->Periodic();
  if (autonomy.Periodic()) // Returns true when finished
    this->EndCompetition();
}

void Robot::TeleopInit() {
  // Stops all subsystem motors, and resets all of their variables to their initial states.
  wpi::outs() << "Teleop Init\n";
  Kill();
  teleop.Init();
}

void Robot::TeleopPeriodic() {
  vision.IdentifyTags();
  localization->Periodic();
  teleop.Periodic();
}

void Robot::DisabledInit() {
  // Stops all subsystem motors, and resets all of their variables to their initial states.
  Kill();
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
  // Stops all subsystem motors, and resets all of their variables to their initial states.
    wpi::outs() << "Test Init\n";
  Kill();
  pathingTimer.Stop();
  pathingTimer.Reset();
  localization->addObstacle(Coord{2.5, 1.3}, 0.1);
  // localization->addObstacle(Coord{3.6, 1.4}, 0.2);
  localization->displayGrid(Coord{5.38, 1.5});}

void Robot::TestPeriodic() {
  pathingTimer.Start();
  if (pathingTimer.AdvanceIfElapsed(units::time::second_t{10.0})) {
    Coord targetPos{5.38, 1.35};
    targetPos.x = frc::SmartDashboard::GetNumber("Autonomy/Traversal/Target X", targetPos.x);
    targetPos.y = frc::SmartDashboard::GetNumber("Autonomy/Traversal/Target Y", targetPos.y);
    std::vector<Coord> path = localization->findPath(targetPos);
    wpi::outs() << "\n\nA* Path to " << targetPos.toStr() << " with no obstacles:\n";
    std::string msg = "";
    for (Coord point : path)
      msg = msg + " " + point.toStr() + " ";
    wpi::outs() << "Path has " << std::to_string(path.size()) << " nodes.\n\n";
    wpi::outs() << localization->displayGrid(targetPos, path) << "\n";
  }
}

void Robot::SimulationInit() {
  // Stops all subsystem motors, and resets all of their variables to their initial states.
  Kill();
}

void Robot::SimulationPeriodic() {}

void Robot::Kill() {
  wpi::outs() << "Kill Command Issued\n";
  frc2::CommandScheduler::GetInstance().CancelAll();
  Blinkin::Off();
  mob.Reset();
  exc.Reset();
  hop.Reset();
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif