#pragma once

#include "subsystems/Hopper.h"
#include <frc/smartdashboard/SmartDashboard.h>

HopperSubsystem::HopperSubsystem() {
    hopSpin.RestoreFactoryDefaults();
    Reset();

    frc::SmartDashboard::PutNumber("Hopper Max Speed", maxSpinSpeed);
    frc::SmartDashboard::SetPersistent("Hopper Max Speed");
    
    frc::SmartDashboard::PutData("Hopper Motor", &hopSpin);
}

void HopperSubsystem::Periodic() {}
void HopperSubsystem::SimulationPeriodic() {}

void HopperSubsystem::Reset() {

    hopSpin.StopMotor();

    isSpinning = false;
}

void HopperSubsystem::HoldLock(bool lock) {
    isLocked = lock;
}

void HopperSubsystem::Spin(double speed, bool invert) {
    maxSpinSpeed = frc::SmartDashboard::GetNumber("Hopper Max Speed", maxSpinSpeed);
    wpi::outs() << "Spin Hopper " << (invert ? "Forwards" : "Backwards") << " @ " << std::to_string(speed * 100.0) << "%\n";
    isSpinning = true;

    double pwr = speed * maxSpinSpeed;
    if (!isLocked || abs(hopSpin.Get()) < pwr)
        hopSpin.Set(pwr * (invert ? -1.0 : 1.0));
}

void HopperSubsystem::Stop() {
    if (!isSpinning || isLocked) return;
    wpi::outs() << "Stop Hopper\n";
    isSpinning = false;
    hopSpin.StopMotor();
}