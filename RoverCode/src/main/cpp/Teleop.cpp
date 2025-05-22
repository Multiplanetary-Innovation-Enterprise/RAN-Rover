#include "Teleop.h"
#include <frc2/command/button/Trigger.h>
#include <frc/smartdashboard/SmartDashboard.h>

Teleop::Teleop() {

    frc::SmartDashboard::PutString("Mobility/Mode", "[B] Free Mode");

    frc::SmartDashboard::PutNumber("Primary Controller Left Stick Deadzone", primaryControllerLeftStickDeadzone);
    frc::SmartDashboard::SetPersistent("Primary Controller Left Stick Deadzone");
    frc::SmartDashboard::PutNumber("Primary Controller Right Stick Deadzone", primaryControllerRightStickDeadzone);
    frc::SmartDashboard::SetPersistent("Primary Controller Right Stick Deadzone");
    frc::SmartDashboard::PutNumber("Secondary Controller Left Stick Deadzone", secondaryControllerLeftStickDeadzone);
    frc::SmartDashboard::SetPersistent("Secondary Controller Left Stick Deadzone");
    frc::SmartDashboard::PutNumber("Secondary Controller Right Stick Deadzone", secondaryControllerRightStickDeadzone);
    frc::SmartDashboard::SetPersistent("Secondary Controller Right Stick Deadzone");
}

void Teleop::SetSystems(MobilitySubsystem* mob, ExcavationSubsystem* exc, HopperSubsystem* hop) {
  this->mob = mob;
  this->exc = exc;
  this->hop = hop;
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

std::array<double, 4> steerTargets;
std::array<double, 4> speedFactors;

void Teleop::Mobility() {

    // Left Joystick X and Y on Primary Controller
    double leftStickX = primaryController.GetLeftX();
    double leftStickY = -primaryController.GetLeftY();
    // Left Joystick Unnormalized Magnitude on Primary Controller
    double leftStickMagnitude = std::min(sqrt(leftStickX * leftStickX + leftStickY * leftStickY), 1.0);
    // Left Joystick Angle on Primary Controller
    double leftStickAngle = int(360 + atan2(leftStickX, leftStickY) * 180.0 / M_PI) % 360; // In degrees, 0 is forward, 90 is right, -90 is left, -180/180 is down
    if (leftStickAngle > 180)
      leftStickAngle -= 360;
    leftStickAngle /= 3;

    double leftTrigger = primaryController.GetLeftTriggerAxis();

    // Right Joystick X and Y on Primary Controller
    double rightStickX = primaryController.GetRightX();
    double rightStickY = -primaryController.GetRightY();
    // Left Joystick Unnormalized Magnitude on Primary Controller
    double rightStickMagnitude = std::min(sqrt(rightStickX * rightStickX + rightStickY * rightStickY), 1.0);
    // Right Joystick Angle on Primary Controller
    double rightStickAngle = int(360 + atan2(rightStickX, rightStickY) * 180.0 / M_PI) % 360; // In degrees, 0 is forward, 90 is right, -90 is left, -180/180 is down
    if (rightStickAngle > 180)
      rightStickAngle -= 360;
    rightStickAngle /= 3;

    double rightTrigger = primaryController.GetRightTriggerAxis();

    double dPad = primaryController.GetPOV();
    double aButton = primaryController.GetAButtonPressed();
    double bButton = primaryController.GetBButtonPressed();
    double xButton = primaryController.GetXButtonPressed();
    double yButton = primaryController.GetYButtonPressed();

    /*
      Crawl / Skid [D-Pad Up/Down]

      Ackermann [A Button]

      Free [B Button]

      Crab [X Button]

      ZeroPoint [Y Button]
    */

   // Change Modes
  if (((dPad == 0 || dPad == 180) && mobMode != MobilitySubsystem::MobilityMode::CRAWL) || exc->shouldActuateSlow()) {
    frc::SmartDashboard::PutString("Mobility/Mode", "[D-Pad] Crawl");
    mobMode = MobilitySubsystem::MobilityMode::CRAWL;
    wpi::outs() << "Entering Crawl Mode\nCrawl: Straight Configuration";
    mob->StopAll();
  } else if (aButton && mobMode != MobilitySubsystem::MobilityMode::ACKERMANN && !exc->shouldActuateSlow()) {
    frc::SmartDashboard::PutString("Mobility/Mode", "[A] Ackermann Mode");
    mobMode = MobilitySubsystem::MobilityMode::ACKERMANN;
    wpi::outs() << "Entering Ackermann Mode\n";
    mob->StopAll();
  } else if (bButton && mobMode != MobilitySubsystem::MobilityMode::FREE && !exc->shouldActuateSlow()) {
    frc::SmartDashboard::PutString("Mobility/Mode", "[B] Free Mode");
    mobMode = MobilitySubsystem::MobilityMode::FREE;
    wpi::outs() << "Entering Free Mode\n";
    mob->StopAll();
  } else if (xButton && mobMode != MobilitySubsystem::MobilityMode::CRAB && !exc->shouldActuateSlow()) {
    frc::SmartDashboard::PutString("Mobility/Mode", "[X] Crab Mode");
    mobMode = MobilitySubsystem::MobilityMode::CRAB;
    wpi::outs() << "Entering Crab Mode\n";
    mob->StopAll();
  } else if (yButton && mobMode != MobilitySubsystem::MobilityMode::ZERO_POINT && !exc->shouldActuateSlow()) {
    frc::SmartDashboard::PutString("Mobility/Mode", "[Y] ZeroPoint Mode");
    mobMode = MobilitySubsystem::MobilityMode::ZERO_POINT;
    wpi::outs() << "Entering ZeroPoint Mode\nZeroPoint Steer: Max Angle Configuration\n";
    mob->StopAll();
  }

  // Mode Steering
  if (mobMode == MobilitySubsystem::MobilityMode::CRAWL) {
    steerTargets[0] = 0.0;
    steerTargets[1] = 0.0;
    steerTargets[2] = 0.0;
    steerTargets[3] = 0.0;
  } else if (mobMode == MobilitySubsystem::MobilityMode::ACKERMANN && leftStickMagnitude > 0.5) {
    // wpi::outs() << "Ackermann Steer: " << std::to_string(leftStickAngle) << " degrees\n";

    double leftAngle = fabs(leftStickAngle) / 180.0 * M_PI;
    double centerOffset = (40.0 + (181.0 / 6.0) * tan(leftAngle)) / tan(leftAngle);
    centerOffset = std::max(106.0, std::min(centerOffset, 406.0));
    double closeAngle = atan(40.0 / (centerOffset - (181.0 / 6.0))) * 180.0 / M_PI;
    double farAngle = atan(40.0 / (centerOffset + (181.0 / 6.0))) * 180.0 / M_PI;
    closeAngle = std::max(-28.0, std::min(closeAngle, 28.0));
    farAngle = std::max(-28.0, std::min(farAngle, 28.0));

    double factor = cos((closeAngle - farAngle) / 180.0 * M_PI);

    if (leftStickAngle > 6.0) { // Pushing right
      steerTargets[0] = farAngle;
      steerTargets[1] = -closeAngle;
      steerTargets[2] = farAngle;
      steerTargets[3] = -closeAngle;
      speedFactors = std::array<double, 4>{1.0, factor, 1.0, factor};
    } else if (leftStickAngle < -6.0) { // Pushing left
      steerTargets[0] = -closeAngle;
      steerTargets[1] = farAngle;
      steerTargets[2] = -closeAngle;
      steerTargets[3] = farAngle;
      speedFactors = std::array<double, 4>{factor, 1.0, factor, 1.0};
    } else {
      steerTargets[0] = 0.0;
      steerTargets[1] = 0.0;
      steerTargets[2] = 0.0;
      steerTargets[3] = 0.0;
    }

  } else if (mobMode == MobilitySubsystem::MobilityMode::FREE) {

    if (leftStickMagnitude > 0.5)
      leftStickAngle = std::max(-28.0, std::min(leftStickAngle, 28.0));
    if (rightStickMagnitude > 0.5)
      rightStickAngle = std::max(-28.0, std::min(rightStickAngle, 28.0));  

    if (leftStickMagnitude > 0.5 || rightStickMagnitude > 0.5) {    
      if (fabs(leftStickAngle) > 2.0 && fabs(rightStickAngle) > 2.0 && leftStickAngle / rightStickAngle < 0) { // Smart Ackermann
        // Documentation for this math is in the RAN Google Drive > "Math for General Ackermann"
        // Partial credit goes to Paul R. Badour for assistance in solving this problem.
        double frontInner = fabs(leftStickAngle) / 180.0 * M_PI; // To radians
        double backInner = fabs(rightStickAngle) / 180.0 * M_PI; // To radians
        double slopeF = tan(frontInner);
        double slopeB = tan(backInner);
        double denom = 80 / (80 + (60 + (1.0/3.0)) * (slopeF + slopeB));
        double frontOuter = atan(slopeF / denom);
        double backOuter = atan(slopeB / denom);

        // Form a right triangle with the hypotenuse being the inner wheel and the outer wheel being the longer of the two other sides.
        // Using the difference between the two steer angles we can find the angle between the hypotenuse and the long side.
        // With this if we let the long side be length 1, we can then calculate the ratio from 1 to the hypotenuse length with cosine.
        // This is the factor we multiply the inner wheel speed by since they need to spin slightly slower than the outer wheels.
        double frontFactor = cos(frontInner - frontOuter);
        double backFactor = cos(backInner - backOuter);

        if (leftStickAngle > 0) {
          steerTargets[0] = frontOuter * 180.0 / M_PI; // To degrees
          steerTargets[1] = -frontInner * 180.0 / M_PI;
          steerTargets[2] = backOuter * 180.0 / M_PI;
          steerTargets[3] = -backInner * 180.0 / M_PI;
          speedFactors = std::array<double, 4>{1.0, frontFactor, 1.0, backFactor};
        } else if (leftStickAngle < 0) {
          steerTargets[0] = -frontInner * 180.0 / M_PI; // To degrees
          steerTargets[1] = frontOuter * 180.0 / M_PI;
          steerTargets[2] = -backInner * 180.0 / M_PI;
          steerTargets[3] = backOuter * 180.0 / M_PI;
          speedFactors = std::array<double, 4>{frontFactor, 1.0, backFactor, 1.0};
        }

      } else { // Crab Mode
        steerTargets[0] = leftStickAngle;
        steerTargets[1] = -leftStickAngle;
        steerTargets[2] = -rightStickAngle;
        steerTargets[3] = rightStickAngle;
      }
    }
    // wpi::outs() << "Free Steer: " << std::to_string(steerTargets[0]) << " degrees, " << std::to_string(steerTargets[2]) << " degrees\n";
  } else if (mobMode == MobilitySubsystem::MobilityMode::CRAB && leftStickMagnitude > 0.5) {
    // wpi::outs() << "Crab Steer: " << std::to_string(leftStickAngle) << " degrees\n";
    leftStickAngle = std::max(-28.0, std::min(leftStickAngle, 28.0));
    steerTargets[0] = leftStickAngle;
    steerTargets[1] = -leftStickAngle;
    steerTargets[2] = -leftStickAngle;
    steerTargets[3] = leftStickAngle;
  } else if (mobMode == MobilitySubsystem::MobilityMode::ZERO_POINT) {
    steerTargets[0] = 28.0;
    steerTargets[1] = 28.0;
    steerTargets[2] = 58.0;
    steerTargets[3] = 58.0;
  }

  bool finishedSteering = mob->SteerTo(steerTargets);

  // Mode Driving
  if (mobMode == MobilitySubsystem::MobilityMode::CRAWL) {

    // D-Pad Up is forward
    // D-Pad Down is backwards
    if (dPad == 0 && finishedSteering) {
      mob->Crawl(true);
      Blinkin::Set(BlinkinPattern::HEARTBEAT_GRAY);
    } else if (dPad == 180 && finishedSteering) {
      mob->Crawl(false);
      Blinkin::Set(BlinkinPattern::HEARTBEAT_GRAY);
    } else {
      mob->StopAll();
      Blinkin::Set(BlinkinPattern::GRAY);
    }

  } else if (mobMode == MobilitySubsystem::MobilityMode::ACKERMANN) {

    // Right Trigger is forward
    // Left Trigger is backward
    double ackermannSpeed = rightTrigger - leftTrigger;
    if (fabs(ackermannSpeed) > 0.01 && finishedSteering) {
      Blinkin::Set(BlinkinPattern::LAWN_GREEN);
      mob->Drive({ackermannSpeed * speedFactors[0], ackermannSpeed * speedFactors[1], ackermannSpeed * speedFactors[2], ackermannSpeed * speedFactors[3]});
    } else {
      Blinkin::Set(BlinkinPattern::GREEN);
      mob->Drive({0.0, 0.0, 0.0, 0.0});
    }

  } else if (mobMode == MobilitySubsystem::MobilityMode::FREE) {

    // Right Trigger is forward
    // Left Trigger is backward
    double freeSpeed = rightTrigger - leftTrigger;
    if (fabs(freeSpeed) > 0.01) {
      Blinkin::Set(BlinkinPattern::RED_ORANGE);
      mob->Drive({freeSpeed * speedFactors[0], freeSpeed * speedFactors[1], freeSpeed * speedFactors[2], freeSpeed * speedFactors[3]});
    } else {
      Blinkin::Set(BlinkinPattern::RED);
      mob->Drive({0.0, 0.0, 0.0, 0.0});
    }

  } else if (mobMode == MobilitySubsystem::MobilityMode::CRAB) {

    // Right Trigger is forward
    // Left Trigger is backward
    double crabSpeed = rightTrigger - leftTrigger;
    if (fabs(crabSpeed) > 0.01 && finishedSteering) {
      Blinkin::Set(BlinkinPattern::BLUE_VIOLET);
      mob->Drive({crabSpeed, crabSpeed, crabSpeed, crabSpeed});
    } else {
      Blinkin::Set(BlinkinPattern::BLUE);
      mob->Drive({0.0, 0.0, 0.0, 0.0});
    }

  } else if (mobMode == MobilitySubsystem::MobilityMode::ZERO_POINT) {

    if (!finishedSteering)
      return;
    // Right Trigger is clockwise
    // Left Trigger is counter-clockwise
    double zeroPointSpeed = rightTrigger - leftTrigger;
    if (fabs(zeroPointSpeed) > 0.01) {
      Blinkin::Set(BlinkinPattern::GOLD);
      mob->Drive({zeroPointSpeed * 0.55, -zeroPointSpeed * 0.55, zeroPointSpeed, -zeroPointSpeed});
    } else {
      Blinkin::Set(BlinkinPattern::YELLOW);
      mob->Drive({0.0, 0.0, 0.0, 0.0});
    }
  }
}

void Teleop::Excavation() {

    // Right Trigger on Secondary Controller
    double secondaryControllerRightTrigger = secondaryController.GetRightTriggerAxis();

    // If there is an input from the right trigger, spin the Excavator.
    if (secondaryControllerRightTrigger > 0.1) {
      // Excavation Spin takes a boolean to determine the direction to spin.
      exc->Spin(secondaryControllerRightTrigger, secondaryController.GetRightBumper());
    } else {
      exc->Spin(0.0, false);
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
      exc->StopActuate();
    }
}

void Teleop::Hopper() {

    // Left Trigger on Secondary Controller
    double secondaryControllerLeftTrigger = secondaryController.GetLeftTriggerAxis();

    // Primary controller indexes hopper during excavation.
    if (primaryController.GetBackButton() || primaryController.GetStartButton()) {
      hop->Spin(1.0, primaryController.GetBackButton(), true);
    } else if (secondaryControllerLeftTrigger > 0.0) { // If there is an input from the left trigger, spin the Hopper.
      // Hopper Spin takes a boolean to determine the direction to spin.
      hop->Spin(secondaryControllerLeftTrigger, secondaryController.GetLeftBumper(), false);
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