#include "Teleop.h"
#include <frc2/command/button/Trigger.h>
#include <frc/smartdashboard/SmartDashboard.h>

Teleop::Teleop(Robot* robot, MobilitySubsystem* mob, ExcavationSubsystem* exc, HopperSubsystem* hop)
    : robot(robot), mob(mob), exc(exc), hop(hop) {

    frc::SmartDashboard::PutNumber("Primary Controller Left Stick Deadzone", primaryControllerLeftStickDeadzone);
    frc::SmartDashboard::SetPersistent("Primary Controller Left Stick Deadzone");
    frc::SmartDashboard::PutNumber("Primary Controller Right Stick Deadzone", primaryControllerRightStickDeadzone);
    frc::SmartDashboard::SetPersistent("Primary Controller Right Stick Deadzone");
    frc::SmartDashboard::PutNumber("Secondary Controller Left Stick Deadzone", secondaryControllerLeftStickDeadzone);
    frc::SmartDashboard::SetPersistent("Secondary Controller Left Stick Deadzone");
    frc::SmartDashboard::PutNumber("Secondary Controller Right Stick Deadzone", secondaryControllerRightStickDeadzone);
    frc::SmartDashboard::SetPersistent("Secondary Controller Right Stick Deadzone");
}

void Teleop::Init() {

    primaryControllerLeftStickDeadzone = frc::SmartDashboard::GetNumber("Primary Controller Left Stick Deadzone", primaryControllerLeftStickDeadzone);
    primaryControllerRightStickDeadzone = frc::SmartDashboard::GetNumber("Primary Controller Right Stick Deadzone", primaryControllerRightStickDeadzone);
    secondaryControllerLeftStickDeadzone = frc::SmartDashboard::GetNumber("Secondary Controller Left Stick Deadzone", secondaryControllerLeftStickDeadzone);
    secondaryControllerRightStickDeadzone = frc::SmartDashboard::GetNumber("Secondary Controller Right Stick Deadzone", secondaryControllerRightStickDeadzone);
}

void Teleop::Periodic() {

    Mobility();
    Excavation();
    Hopper();
}

void Teleop::Mobility() {

    // Left Joystick X on Primary Controller
    double leftStickX = primaryController.GetLeftX();
    // Left Joystick Y on Primary Controller
    double leftStickY = -primaryController.GetLeftY();
    // Left Joystick Unnormalized Magnitude on Primary Controller
    double leftStickMagnitude = std::min(sqrt(leftStickX * leftStickX + leftStickY * leftStickY), 1.0);
    // Left Joystick Angle on Primary Controller
    double leftStickAngle = int(360 + atan2(leftStickX, leftStickY) * 180.0 / M_PI) % 360; // In degrees, 0 is forward, 90 is right, 180 is backward, 270 is left

    // Right Joystick X and Y on Primary Controller
    double rightStickX = primaryController.GetRightX();
    double rightStickY = -primaryController.GetRightY();
    // Right Joystick Angle on Primary Controller
    double rightStickAngle = int(360 + atan2(rightStickX, rightStickY) * 180.0 / M_PI) % 360; // In degrees, 0 is forward, 90 is right, 180 is backward, 270 is left

    // D-Pad on Controller for Crawl (Slow drive)
    if (primaryController.GetPOV() == 0) { // D-Pad Up
      mob->Crawl(true);
    } else if (primaryController.GetPOV() == 180) { // D-Pad Down
      mob->Crawl(false);
    } else {
      // If not crawling, allow for normal stick input.

      // If there is a stick input, drive, otherwise, stop driving.
      if (leftStickMagnitude > primaryControllerLeftStickDeadzone) {

        // Stick zone for negation of magnitude
        if (leftStickAngle > 120 && leftStickAngle < 240)
          leftStickMagnitude *= -1;

        wpi::outs() << "Steer Angle: " << std::to_string(int(leftStickAngle)) << "\n";
        // Mobility Drive takes the left side input, right side input, and a boolean to determine if it should crawl
        mob->Drive({leftStickMagnitude, leftStickMagnitude, leftStickMagnitude, leftStickMagnitude});
      } else {
        mob->StopAll();
      }
    }
}

void Teleop::Excavation() {

    // Right Trigger on Secondary Controller
    double secondaryControllerRightTrigger = secondaryController.GetRightTriggerAxis();

    // If there is an input from the right trigger, spin the Excavator.
    if (secondaryControllerRightTrigger > 0.0) {
      // Excavation Spin takes a boolean to determine the direction to spin.
      exc->Spin(secondaryControllerRightTrigger, secondaryController.GetRightBumper());
    } else {
      exc->Stop();
    }
    
    // Holding B Button Locks the current speed and direction of the Excavator.
    if (secondaryController.GetBButtonPressed()) {
      exc->HoldLock(true);
    } else if (secondaryController.GetBButtonReleased()) {
      exc->HoldLock(false);
    }

    if (!secondaryController.GetYButton() != !secondaryController.GetAButton()) { // XOR, only actuate when one is pressed.
      exc->StartActuate(secondaryController.GetAButton());
    } else {
      exc->StopActuate(false);
    }
}

void Teleop::Hopper() {

    // Left Trigger on Secondary Controller
    double secondaryControllerLeftTrigger = secondaryController.GetLeftTriggerAxis();

    // If there is an input from the left trigger, spin the Hopper.
    if (secondaryControllerLeftTrigger > 0.0) {
      // Hopper Spin takes a boolean to determine the direction to spin.
      hop->Spin(secondaryControllerLeftTrigger, secondaryController.GetLeftBumper());
    } else {
      hop->Stop();
    }
    
    // Holding X Button Locks the current speed and direction of the Hopper.
    if (secondaryController.GetXButtonPressed()) {
      hop->HoldLock(true);
    } else if (secondaryController.GetXButtonReleased()) {
      hop->HoldLock(false);
    }
}