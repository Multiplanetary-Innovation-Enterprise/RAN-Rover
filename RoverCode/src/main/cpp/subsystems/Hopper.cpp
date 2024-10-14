#pragma once

#include "subsystems/Hopper.h"

HopperSubsystem::HopperSubsystem() {
    hopSpin.RestoreFactoryDefaults();
    Reset();
}

void HopperSubsystem::Periodic() {}
void HopperSubsystem::SimulationPeriodic() {}

void HopperSubsystem::Reset() {

    hopSpin.StopMotor();

    isSpinning = false;
}

void HopperSubsystem::Spin(double speed, bool invert) {
    wpi::outs() << "Spin Hopper " << (invert ? "Forwards" : "Backwards") << " @ " << std::to_string(speed * 100.0) << "%\n";
    isSpinning = true;
    hopSpin.Set(speed * maxSpinSpeed * (invert ? -1.0 : 1.0));
}

void HopperSubsystem::Stop() {
    if (!isSpinning) return;
    wpi::outs() << "Stop Hopper\n";
    isSpinning = false;
    hopSpin.StopMotor();
}