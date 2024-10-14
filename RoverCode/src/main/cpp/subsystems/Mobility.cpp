#pragma once

#include "subsystems/Mobility.h"

MobilitySubsystem::MobilitySubsystem() {
    backLeft.RestoreFactoryDefaults();
    frontLeft.RestoreFactoryDefaults();
    backRight.RestoreFactoryDefaults();
    frontRight.RestoreFactoryDefaults();

    backLeft.SetInverted(true);
    frontLeft.SetInverted(true);

    slipWait[0] = 0; // Back Left
    slipWait[1] = 0; // Front Left
    slipWait[2] = 0; // Back Right
    slipWait[3] = 0; // Front Right

    Reset();
}

void MobilitySubsystem::Periodic() {}
void MobilitySubsystem::SimulationPeriodic() {}

void MobilitySubsystem::Reset() {

    backLeft.StopMotor();
    frontLeft.StopMotor();
    backRight.StopMotor();
    frontRight.StopMotor();

    isSpinning = false;
    leftCrawl = false;
    rightCrawl = false;
}

std::array<double, 4> MobilitySubsystem::SlipControl(std::array<double, 4> in) {
    std::array<double, 4> out = in;

    // Back Left
    if (abs(in[0]) > 0.0) { // Do we want this motor to move?
        if (slipWait[0] >= cyclesSlipRun) { // Checking Stage
            if (backLeft.GetOutputCurrent() >= currentThreshold) { // Slippage detected
                slipWait[0] = -cyclesSlipOff;
            }
        } else {
            if (slipWait[0] < 0) { // Off Stage
                out[0] = 0.0;
            }
            slipWait[0]++;
        }
    }

    return out;
}

void MobilitySubsystem::Drive(double leftSpeed, double rightSpeed, bool crawl) {
    isSpinning = true;
    double crawlFactor = crawl ? maxCrawlSpeed : maxDriveSpeed;
    wpi::outs() << "Start Motors <" << std::to_string(leftSpeed) << ", " << std::to_string(rightSpeed) << "> @ " << std::to_string(crawlFactor * 100.0) << "%\n";
    backLeft.Set(leftSpeed * crawlFactor);
    frontLeft.Set(leftSpeed * crawlFactor);
    backRight.Set(rightSpeed * crawlFactor);
    frontRight.Set(rightSpeed * crawlFactor); 
}

void MobilitySubsystem::Stop() {
    if (!isSpinning) return;
    wpi::outs() << "Stop Motors\n";
    isSpinning = false;
    backLeft.StopMotor();
    frontLeft.StopMotor();
    backRight.StopMotor();
    frontRight.StopMotor();
}