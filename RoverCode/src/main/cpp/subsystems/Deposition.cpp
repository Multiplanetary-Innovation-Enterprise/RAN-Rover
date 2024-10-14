#pragma once

#include "subsystems/Deposition.h"

DepositionSubsystem::DepositionSubsystem() {
    depSpin.RestoreFactoryDefaults();
    Reset();
}

void DepositionSubsystem::Periodic() {}
void DepositionSubsystem::SimulationPeriodic() {}

void DepositionSubsystem::Reset() {

    depSpin.StopMotor();

    isSpinning = false;
}

void DepositionSubsystem::Spin(double speed, bool invert) {
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