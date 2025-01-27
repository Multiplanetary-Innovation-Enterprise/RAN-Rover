#pragma once

#include "subsystems/Mobility.h"
#include <frc/smartdashboard/SmartDashboard.h>

MobilitySubsystem::MobilitySubsystem() {

    motor[FRONT_LEFT].RestoreFactoryDefaults();
    motor[FRONT_RIGHT].RestoreFactoryDefaults();
    motor[BACK_LEFT].RestoreFactoryDefaults();
    motor[BACK_RIGHT].RestoreFactoryDefaults();

    motor[FRONT_RIGHT].SetInverted(true);
    motor[BACK_RIGHT].SetInverted(true);

    slipWait[FRONT_LEFT] = 0; 
    slipWait[FRONT_RIGHT] = 0;
    slipWait[BACK_LEFT] = 0;
    slipWait[BACK_RIGHT] = 0;

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

    frc::SmartDashboard::PutData("Front Left Motor", &motor[FRONT_LEFT]);
    frc::SmartDashboard::PutData("Front Right Motor", &motor[FRONT_RIGHT]);
    frc::SmartDashboard::PutData("Back Left Motor", &motor[BACK_LEFT]);
    frc::SmartDashboard::PutData("Back Right Motor", &motor[BACK_RIGHT]);
    
    Reset();
}

void MobilitySubsystem::Periodic() {}
void MobilitySubsystem::SimulationPeriodic() {}

void MobilitySubsystem::Reset() {
    
    motor[FRONT_LEFT].StopMotor();
    motor[FRONT_RIGHT].StopMotor();
    motor[BACK_LEFT].StopMotor();
    motor[BACK_RIGHT].StopMotor();

    isSpinning = false;
    leftCrawl = false;
    rightCrawl = false;
}

std::array<double, 4> MobilitySubsystem::SlipControl(std::array<double, 4> in) {
    std::array<double, 4> out = in;

    for (int i = 0; i < 4; i++) {
        if (abs(in[i]) > 0.0) { // Do we want this motor to move?
            if (slipWait[i] >= cyclesSlipTry) { // Checking Stage
                if (motor[i].GetOutputCurrent() >= currentThreshold) { // Slippage detected
                    slipWait[i] = -cyclesSlipOff;
                }
            } else {
                if (slipWait[i] < 0) { // Off Stage
                    out[i] = 0.0;
                }
                slipWait[i]++;
            }
        }
    }

    return out;
}

void MobilitySubsystem::Drive(std::array<double, 4> speed) {
    maxDriveSpeed = frc::SmartDashboard::GetNumber("Drive Max Speed", maxDriveSpeed);

    isSpinning = true;
    wpi::outs() << "Drive (" << std::to_string(int(maxDriveSpeed * 100.0)) << "%) | Raw Speed: <";
    for (int i = 0; i < 4; i++)
        wpi::outs() << std::to_string(int(speed[i] * 100.0)) << "%" << (i < 4 ? ", " : ">");

    std::array<double, 4> controlledSpeed = SlipControl(speed);
    wpi::outs() << " | Slip Controlled Speed: <";
    for (int i = 0; i < 4; i++) {
        wpi::outs() << std::to_string(int(controlledSpeed[i] * 100.0)) << "%" << (i < 4 ? ", " : ">\n");
        motor[i].Set(controlledSpeed[i] * maxDriveSpeed);
    }
}

void MobilitySubsystem::Crawl(bool forward) {
    maxCrawlSpeed = frc::SmartDashboard::GetNumber("Crawl Max Speed", maxCrawlSpeed);

    isSpinning = true;
    wpi::outs() << "Crawl (" << std::to_string(int(maxCrawlSpeed * 100.0)) << "%)\n";
    for (int i = 0; i < 4; i++)
        motor[i].Set(maxCrawlSpeed * (forward ? 1 : -1));
}

void MobilitySubsystem::StopAll() {
    if (!isSpinning) return;
    wpi::outs() << "Stop Motors\n";
    isSpinning = false;
    motor[FRONT_LEFT].StopMotor();
    motor[FRONT_RIGHT].StopMotor();
    motor[BACK_LEFT].StopMotor();
    motor[BACK_RIGHT].StopMotor();
}

void MobilitySubsystem::StartActuate(Wheel w, bool dir) {

    if (dir) {
        if (actuateDirs[w] == 1) {
            if (actPot[w].Get() >= actuateLimits[w][0] || actPot[w].Get() >= actuateLimits[w][1])
                StopActuate(w, true);
            return;
        }
        actuateDirs[w] = 1;
    } else {
        if (actuateDirs[w] == -1) {
            if (actPot[w].Get() <= actuateLimits[w][1] || actPot[w].Get() <= actuateLimits[w][0])
                StopActuate(w, true);
            return;
        }
        actuateDirs[w] = -1;
    }

    actDir[w].Set(dir);
    actVel[w].Set(true);
}

void MobilitySubsystem::StopActuate(Wheel w, bool limitHit) {
    if (actuateDirs[w] == 0)
        return;
    if (!limitHit)
        actuateDirs[w] = 0;
    actVel[w].Set(false);
}

double MobilitySubsystem::GetAngle(Wheel w) {
    return (actPot[w].Get() - actuateLimits[w][0]) / (actuateLimits[w][1] - actuateLimits[w][0]) * 90.0 / 45.0;
}