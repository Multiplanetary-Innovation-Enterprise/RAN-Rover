#pragma once

#include "subsystems/Excavation.h"

ExcavationSubsystem::ExcavationSubsystem() {
    excSpin.RestoreFactoryDefaults();
    Reset();
}

void ExcavationSubsystem::Periodic() {}
void ExcavationSubsystem::SimulationPeriodic() {}

void ExcavationSubsystem::Reset() {

    excSpin.StopMotor();

    isSpinning = false;
}

void ExcavationSubsystem::Spin(double speed, bool invert) {
    wpi::outs() << "Spin Excavation " << (invert ? "Forwards" : "Backwards") << " @ " << std::to_string(speed * 100.0) << "%\n";
    isSpinning = true;
    excSpin.Set(speed * maxSpinSpeed * (invert ? -1 : 1));
}

void ExcavationSubsystem::Stop() {
    if (!isSpinning) return;
    wpi::outs() << "Stop Excavation\n";
    isSpinning = false;
    excSpin.StopMotor();
}