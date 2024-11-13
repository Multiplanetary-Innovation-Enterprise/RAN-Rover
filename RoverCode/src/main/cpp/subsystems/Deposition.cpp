#pragma once

#include "subsystems/Deposition.h"
#include <frc/smartdashboard/SmartDashboard.h>

DepositionSubsystem::DepositionSubsystem() {
    depSpin.RestoreFactoryDefaults();
    Reset();

    frc::SmartDashboard::PutNumber("Deposition Max Speed", maxSpinSpeed);
    frc::SmartDashboard::SetPersistent("Deposition Max Speed");
    
    frc::SmartDashboard::PutNumber("Deposition Actuate Speed", actuateSpeed);
    frc::SmartDashboard::SetPersistent("Deposition Actuate Speed");
    frc::SmartDashboard::PutNumber("Deposition Spin Threshold", actuateSpinThreshold);
    frc::SmartDashboard::SetPersistent("Deposition Spin Threshold");
    frc::SmartDashboard::PutNumber("Deposition Actuate Min", actuateMin);
    frc::SmartDashboard::SetPersistent("Deposition Actuate Min");
    frc::SmartDashboard::PutNumber("Deposition Actuate Max", actuateMax);
    frc::SmartDashboard::SetPersistent("Deposition Actuate Max");
    
    frc::SmartDashboard::PutData("Deposition Motor", &depSpin);
    frc::SmartDashboard::PutData("Deposition Left Actuator Pot", &depLeftPot);
    frc::SmartDashboard::PutData("Deposition Right Actuator Pot", &depRightPot);
}

void DepositionSubsystem::Periodic() {}
void DepositionSubsystem::SimulationPeriodic() {}

void DepositionSubsystem::Reset() {

    depSpin.StopMotor();
    isSpinning = false;
}

bool DepositionSubsystem::canSpin() {
    actuateSpinThreshold = frc::SmartDashboard::GetNumber("Deposition Spin Threshold", actuateSpinThreshold);
    return depLeftPot.Get() >= actuateSpinThreshold && depRightPot.Get() >= actuateSpinThreshold;
}

void DepositionSubsystem::Spin(double speed, bool invert) {
    maxSpinSpeed = frc::SmartDashboard::GetNumber("Deposition Max Speed", maxSpinSpeed);
    if (!canSpin()) {
        Stop();
        return;
    }
    wpi::outs() << "Spin Deposition " << (invert ? "Forwards" : "Backwards") << " @ " << std::to_string(speed * 100.0) << "%\n";
    isSpinning = true;
    depSpin.Set(speed * maxSpinSpeed * (invert ? -1 : 1));
}
 
void DepositionSubsystem::Stop() {
    if (!isSpinning) return; 
    wpi::outs() << "Stop Deposition\n";
    isSpinning = false;
    depSpin.StopMotor();
}

void DepositionSubsystem::StartActuate(bool dir) {
    if (dir) {
        actuateMax = frc::SmartDashboard::GetNumber("Deposition Actuate Max", actuateMax);
        if (actuatingDir == 1) {
            if (depLeftPot.Get() >= actuateMax || depRightPot.Get() >= actuateMax) StopActuate(true);
            return;
        }
        wpi::outs() << "Deploying Deposition...\n";
        actuatingDir = 1;
    } else {
        actuateMin = frc::SmartDashboard::GetNumber("Deposition Actuate Min", actuateMin);
        if (actuatingDir == -1) {
            if (depLeftPot.Get() <= actuateMin || depRightPot.Get() <= actuateMin) StopActuate(true);
            return;
        }
        wpi::outs() << "Retracting Deposition...\n";
        actuatingDir = -1;
    }

    depLeftActDir.Set(dir);
    depRightActDir.Set(dir);
    
    // Variable linear actuator speed not currently functional, WIP!
    actuateSpeed = frc::SmartDashboard::GetNumber("Deposition Actuate Speed", actuateSpeed);
    depLeftActVel.SetPWMRate(actuateSpeed);
    depRightActVel.SetPWMRate(actuateSpeed);
    depLeftActVel.Set(true);
    depRightActVel.Set(true);
}

void DepositionSubsystem::StopActuate(bool limitHit) {
    if (actuatingDir == 0) return;
    wpi::outs() << "Stopping Actuating Deposition\n";
    if (!limitHit)
        actuatingDir = 0;
    depLeftActVel.Set(false);
    depRightActVel.Set(false);
}