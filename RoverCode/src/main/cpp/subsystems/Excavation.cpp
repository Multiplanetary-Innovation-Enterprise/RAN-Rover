#pragma once

#include "subsystems/Excavation.h"
#include <frc/smartdashboard/SmartDashboard.h>

ExcavationSubsystem::ExcavationSubsystem() {
    excSpin.RestoreFactoryDefaults();
    Reset();

    frc::SmartDashboard::PutNumber("Excavation Max Speed", maxSpinSpeed);
    frc::SmartDashboard::SetPersistent("Excavation Max Speed");

    frc::SmartDashboard::PutNumber("Excavation Actuate Speed", actuateSpeed);
    frc::SmartDashboard::SetPersistent("Excavation Actuate Speed");
    frc::SmartDashboard::PutNumber("Excavation Spin Threshold", actuateSpinThreshold);
    frc::SmartDashboard::SetPersistent("Excavation Spin Threshold");
    frc::SmartDashboard::PutNumber("Excavation Actuate Min", actuateMin);
    frc::SmartDashboard::SetPersistent("Excavation Actuate Min");
    frc::SmartDashboard::PutNumber("Excavation Actuate Max", actuateMax);
    frc::SmartDashboard::SetPersistent("Excavation Actuate Max");

    frc::SmartDashboard::PutData("Excavation Motor", &excSpin);
    frc::SmartDashboard::PutData("Excavation Left Actuator Pot", &excLeftPot);
    frc::SmartDashboard::PutData("Excavation Right Actuator Pot", &excRightPot);
}

void ExcavationSubsystem::Periodic() {}
void ExcavationSubsystem::SimulationPeriodic() {}

void ExcavationSubsystem::Reset() {

    excSpin.StopMotor();

    isSpinning = false;
    actuatingDir = 0;
    isLocked = false;
}

bool ExcavationSubsystem::canSpin() {
    actuateSpinThreshold = frc::SmartDashboard::GetNumber("Excavation Spin Threshold", actuateSpinThreshold);
    return excLeftPot.Get() >= actuateSpinThreshold && excRightPot.Get() >= actuateSpinThreshold;
}

void ExcavationSubsystem::HoldLock(bool lock) {
    isLocked = lock;
}

void ExcavationSubsystem::Spin(double speed, bool invert) {

    maxSpinSpeed = frc::SmartDashboard::GetNumber("Excavation Max Speed", maxSpinSpeed);
    if (!canSpin()) {
        Stop();
        return;
    }
    double pwr = speed * maxSpinSpeed;
    if (!isLocked || abs(excSpin.Get()) < pwr) {
        isSpinning = true;
        wpi::outs() << "Spin Excavation " << (invert ? "Forwards" : "Backwards") << " @ " << std::to_string(speed * 100.0) << "%\n";
        excSpin.Set(pwr * (invert ? -1.0 : 1.0));
    }
}

void ExcavationSubsystem::Stop() {
    if (!isSpinning || isLocked) return;
    wpi::outs() << "Stop Excavation\n";
    isSpinning = false;
    excSpin.StopMotor();
}

void ExcavationSubsystem::StartActuate(bool dir) {
    if (dir) {
        actuateMax = frc::SmartDashboard::GetNumber("Excavation Actuate Max", actuateMax);
        if (actuatingDir == 1) {
            if (excLeftPot.Get() >= actuateMax || excRightPot.Get() >= actuateMax) StopActuate(true);
            return;
        }
        wpi::outs() << "Deploying Excavator...\n";
        actuatingDir = 1;
    } else {
        actuateMin = frc::SmartDashboard::GetNumber("Excavation Actuate Min", actuateMin);
        if (actuatingDir == -1) {
            if (excLeftPot.Get() <= actuateMin || excRightPot.Get() <= actuateMin) StopActuate(true);
            return;
        }
        wpi::outs() << "Retracting Excavator...\n";
        actuatingDir = -1;
    }
    
    excLeftActDir.Set(dir);
    excRightActDir.Set(dir);

    // Variable linear actuator speed not currently functional, WIP!
    actuateSpeed = frc::SmartDashboard::GetNumber("Excavation Actuate Speed", actuateSpeed);
    excLeftActVel.SetPWMRate(actuateSpeed);
    excRightActVel.SetPWMRate(actuateSpeed);
    excLeftActVel.Set(true);
    excRightActVel.Set(true);
}

void ExcavationSubsystem::StopActuate(bool limitHit) {
    if (actuatingDir == 0) return;
    wpi::outs() << "Stopping Actuating Excavator\n";
    if(!limitHit)
        actuatingDir = 0;
    excLeftActVel.Set(false);
    excRightActVel.Set(false);
}