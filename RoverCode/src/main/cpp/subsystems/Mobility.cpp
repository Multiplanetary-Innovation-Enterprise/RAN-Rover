#pragma once

#include "subsystems/Mobility.h"
#include <frc/smartdashboard/SmartDashboard.h>

MobilitySubsystem::MobilitySubsystem() {
    backLeft.RestoreFactoryDefaults();
    frontLeft.RestoreFactoryDefaults();
    backRight.RestoreFactoryDefaults();
    frontRight.RestoreFactoryDefaults();

    backRight.SetInverted(true);
    frontRight.SetInverted(true);

    slipWait[0] = 0; // Back Left
    slipWait[1] = 0; // Front Left
    slipWait[2] = 0; // Back Right
    slipWait[3] = 0; // Front Right

    frc::SmartDashboard::PutNumber("Drive Max Speed", maxDriveSpeed);
    frc::SmartDashboard::SetPersistent("Drive Max Speed");
    frc::SmartDashboard::PutNumber("Crawl Max Speed", maxCrawlSpeed);
    frc::SmartDashboard::SetPersistent("Crawl Max Speed");

    frc::SmartDashboard::PutNumber("Slip Current Threshold", currentThreshold);
    frc::SmartDashboard::SetPersistent("Slip Current Threshold");
    frc::SmartDashboard::PutNumber("Slip Off Cycles", cyclesSlipOff);
    frc::SmartDashboard::SetPersistent("Slip Off Cycles");
    frc::SmartDashboard::PutNumber("Slip Try Cycles", cyclesSlipTry);
    frc::SmartDashboard::SetPersistent("Slip Try Cycles");

    frc::SmartDashboard::PutData("Front Left Motor", &frontLeft);
    frc::SmartDashboard::PutData("Front Right Motor", &frontRight);
    frc::SmartDashboard::PutData("Back Left Motor", &backLeft);
    frc::SmartDashboard::PutData("Back Right Motor", &backRight);
    
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
        if (slipWait[0] >= cyclesSlipTry) { // Checking Stage
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
    maxCrawlSpeed = frc::SmartDashboard::GetNumber("Crawl Max Speed", maxCrawlSpeed);
    maxDriveSpeed = frc::SmartDashboard::GetNumber("Drive Max Speed", maxDriveSpeed);

    isSpinning = true;
    double crawlFactor = crawl ? maxCrawlSpeed : maxDriveSpeed;
    wpi::outs() << "Start Motors <" << std::to_string(int(leftSpeed * 100.0)) << "%, " << std::to_string(int(rightSpeed * 100.0)) << "%> @ " << std::to_string(int(crawlFactor * 100.0)) << "%\n";
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