#include "subsystems/Hopper.h"
#include <frc/smartdashboard/SmartDashboard.h>

HopperSubsystem::HopperSubsystem() {
    hopSpin.RestoreFactoryDefaults();
    Reset();

    frc::SmartDashboard::PutNumber("Hopper/Empty Speed", emptySpinSpeed);
    frc::SmartDashboard::SetPersistent("Hopper/Empty Speed");
    frc::SmartDashboard::PutNumber("Hopper/Fill Speed", fillSpinSpeed);
    frc::SmartDashboard::SetPersistent("Hopper/Fill Speed");
    
    frc::SmartDashboard::PutData("Hopper/Motor", &hopSpin);
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

void HopperSubsystem::Spin(double speed, bool invert, bool filling) {
    
    wpi::outs() << "Spin Hopper " << (invert ? "Forwards" : "Backwards") << " @ " << std::to_string(speed * 100.0) << "%\n";
    isSpinning = true;
    double pwr = speed;

    if (filling) {
        fillSpinSpeed = frc::SmartDashboard::GetNumber("Hopper/Fill Speed", fillSpinSpeed);
        pwr *= fillSpinSpeed;
    } else {
        emptySpinSpeed = frc::SmartDashboard::GetNumber("Hopper/Empty Speed", emptySpinSpeed);
        pwr *= emptySpinSpeed;
    }

    if (!isLocked || fabs(hopSpin.Get()) < pwr)
        hopSpin.Set(pwr * (invert ? -1.0 : 1.0));
}

void HopperSubsystem::Stop() {
    if (!isSpinning || isLocked) return;
    wpi::outs() << "Stop Hopper\n";
    isSpinning = false;
    hopSpin.StopMotor();
}