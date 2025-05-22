#include "subsystems/Excavation.h"
#include <frc/smartdashboard/SmartDashboard.h>

ExcavationSubsystem::ExcavationSubsystem() {
    excSpin.RestoreFactoryDefaults();
    Reset();

    frc::SmartDashboard::PutNumber("Excavation/Steady Speed", steadySpinSpeed);
    frc::SmartDashboard::SetPersistent("Excavation/Steady Speed");
    frc::SmartDashboard::PutNumber("Excavation/Plunge Speed", plungeSpinSpeed);
    frc::SmartDashboard::SetPersistent("Excavation/Plunge Speed");

    frc::SmartDashboard::PutNumber("Excavation/Spin Threshold", actuateSpinThreshold);
    frc::SmartDashboard::SetPersistent("Excavation/Spin Threshold");
    frc::SmartDashboard::PutNumber("Excavation/Actuate Min", actuateMin);
    frc::SmartDashboard::SetPersistent("Excavation/Actuate Min");
    frc::SmartDashboard::PutNumber("Excavation/Actuate Max", actuateMax);
    frc::SmartDashboard::SetPersistent("Excavation/Actuate Max");

    frc::SmartDashboard::PutNumber("Excavation/Pulse Width Frequency", actuatePW);
    frc::SmartDashboard::SetPersistent("Excavation/Pulse Width Frequency");
    frc::SmartDashboard::PutNumber("Excavation/Actuate Fast Speed", actuateFastSpeed);
    frc::SmartDashboard::SetPersistent("Excavation/Actuate Fast Speed");
    frc::SmartDashboard::PutNumber("Excavation/Actuate Slow Speed", actuateSlowSpeed);
    frc::SmartDashboard::SetPersistent("Excavation/Actuate Slow Speed");

    frc::SmartDashboard::PutNumber("Excavation/Actuate Pot Diff", actPotDiff);
    frc::SmartDashboard::SetPersistent("Excavation/Actuate Pot Diff");

    frc::SmartDashboard::PutNumber("Excavation/Acceleration Rate", accelerationRate);
    frc::SmartDashboard::SetPersistent("Excavation/Acceleration Rate");

    frc::SmartDashboard::PutData("Excavation/Motor", &excSpin);
    frc::SmartDashboard::PutData("Excavation/Left Actuator Pot", &excLeftPot);
    frc::SmartDashboard::PutData("Excavation/Right Actuator Pot", &excRightPot);

    frc::SmartDashboard::PutNumber("Excavation/PIDControllers/P Gain", kP);
    frc::SmartDashboard::SetPersistent("Excavation/PIDControllers/P Gain");
    frc::SmartDashboard::PutNumber("Excavation/PIDControllers/I Gain", kI);
    frc::SmartDashboard::SetPersistent("Excavation/PIDControllers/I Gain");
    frc::SmartDashboard::PutNumber("Excavation/PIDControllers/D Gain", kD);
    frc::SmartDashboard::SetPersistent("Excavation/PIDControllers/D Gain");
}

void ExcavationSubsystem::Periodic() {

    if (actuatingDir == 1) {
        if (GetActuateLower() <= actuateMin - 0.01) {
            StopActuate();
        } else {
            excLeftActVel.UpdateDutyCycle(actuateFastSpeed);
            excRightActVel.UpdateDutyCycle(actuateFastSpeed);
        }
    } else if (actuatingDir == -1) {
        if (GetActuateUpper() >= actuateMax + 0.01) {
            StopActuate();
        } else {        
            if (shouldActuateSlow()) {
                excLeftActVel.UpdateDutyCycle(actuateSlowSpeed);
                excRightActVel.UpdateDutyCycle(actuateSlowSpeed);
            } else {
                excLeftActVel.UpdateDutyCycle(actuateFastSpeed);
                excRightActVel.UpdateDutyCycle(actuateFastSpeed);
            }
        }
    }
}
void ExcavationSubsystem::SimulationPeriodic() {}

void ExcavationSubsystem::Reset() {

    excSpin.StopMotor();
    currSpeed = 0.0;

    StopActuate();

    isSpinning = false;
    actuatingDir = 0;
    isLocked = false;
}

bool ExcavationSubsystem::shouldActuateSlow() {
    actuateSpinThreshold = frc::SmartDashboard::GetNumber("Excavation/Spin Threshold", actuateSpinThreshold);
    return GetActuateLower() >= actuateSpinThreshold;
}

void ExcavationSubsystem::HoldLock(bool lock) {
    isLocked = lock;
}

void ExcavationSubsystem::Spin(double speed, bool invert) {

    steadySpinSpeed = frc::SmartDashboard::GetNumber("Excavation/Steady Speed", steadySpinSpeed);
    plungeSpinSpeed = frc::SmartDashboard::GetNumber("Excavation/Plunge Speed", plungeSpinSpeed);
    AccelerationControl(speed * (invert ? -1.0 : 1.0));
    double pwr = currSpeed * (GetActuateUpper() >= actuateMax ? steadySpinSpeed : plungeSpinSpeed);
    if (!isLocked || fabs(excSpin.Get()) < pwr) {
        isSpinning = true;
        // wpi::outs() << "Spin Excavation " << (invert ? "Forwards" : "Backwards") << " @ " << std::to_string(speed * 100.0) << "%\n";
        excSpin.Set(pwr);
    }
}

void ExcavationSubsystem::Stop() {
    if (!isSpinning || isLocked) return;
    // wpi::outs() << "Stop Excavation\n";
    isSpinning = false;
    currSpeed = 0.0;
    excSpin.StopMotor();
}

void ExcavationSubsystem::StartActuate(bool dir) {
    actuateSpinThreshold = frc::SmartDashboard::GetNumber("Excavation/Spin Threshold", actuateSpinThreshold);
    if (dir) {
        actuateMin = frc::SmartDashboard::GetNumber("Excavation/Actuate Min", actuateMin);
        if (GetActuateLower() <= actuateMin) {
            StopActuate();
            return;
        } else if (actuatingDir != 1) {
            // wpi::outs() << "Retracting Excavator...\n";
            actuatingDir = 1;
        } else {
            return;
        }
    } else {
        actuateMax = frc::SmartDashboard::GetNumber("Excavation/Actuate Max", actuateMax);
        if (GetActuateUpper() >= actuateMax) {
            StopActuate();
            return;
        } else if (actuatingDir != -1) {
            // wpi::outs() << "Deploying Excavator...\n";
            actuatingDir = -1;
        } else {
            return;
        }
    }

    excLeftActDir.Set(dir);
    excRightActDir.Set(dir);

    actuatePW = frc::SmartDashboard::GetNumber("Excavation/Pulse Width Frequency", actuatePW);
    excLeftActVel.SetPWMRate(actuatePW);
    excRightActVel.SetPWMRate(actuatePW);

    if (actuatingDir == -1 && GetActuateLower() >= actuateSpinThreshold) {
        actuateSlowSpeed = frc::SmartDashboard::GetNumber("Excavation/Actuate Slow Speed", actuateSlowSpeed);
        excLeftActVel.EnablePWM(actuateSlowSpeed);
        excRightActVel.EnablePWM(actuateSlowSpeed);
    } else {
        actuateFastSpeed = frc::SmartDashboard::GetNumber("Excavation/Actuate Fast Speed", actuateFastSpeed);
        excLeftActVel.EnablePWM(actuateFastSpeed);
        excRightActVel.EnablePWM(actuateFastSpeed);
    }
}

void ExcavationSubsystem::StopActuate() {
    if (actuatingDir == 0) return;
    // wpi::outs() << "Stopping Actuating Excavator\n";
    actuatingDir = 0;
    excLeftActVel.DisablePWM();
    excRightActVel.DisablePWM();
    excLeftActDir.Set(false);
    excRightActDir.Set(false);
}

double ExcavationSubsystem::GetActuateUpper() {
    actPotDiff = frc::SmartDashboard::GetNumber("Excavation/Actuate Pot Diff", actPotDiff);
    return std::max(excLeftPot.Get(), excRightPot.Get() - actPotDiff);
}

double ExcavationSubsystem::GetActuateLower() {
    actPotDiff = frc::SmartDashboard::GetNumber("Excavation/Actuate Pot Diff", actPotDiff);
    return std::min(excLeftPot.Get(), excRightPot.Get() - actPotDiff);
}

double ExcavationSubsystem::GetSpeed() {
    return currSpeed;
}

double exc_err_total = 0.0;
double exc_err_prev = 0.0;
double exc_prev_time = 0.0;

void ExcavationSubsystem::PseudoPID(double targetVelocity) {
    // read PID coefficients from SmartDashboard
    double kP = frc::SmartDashboard::GetNumber("Excavation/PIDControllers/P Gain", 0);
    double kI = frc::SmartDashboard::GetNumber("Excavation/PIDControllers/I Gain", 0);
    double kD = frc::SmartDashboard::GetNumber("Excavation/PIDControllers/D Gain", 0);
    
    timer.Start();
    double curr_time = timer.Get().value();
    double err = targetVelocity - excSpin.encoder.GetVelocity();
    exc_err_total = exc_err_total + err;
    double err_slope = (err - exc_err_prev) / (curr_time  - exc_prev_time);

    exc_prev_time = curr_time;
    exc_err_prev = err;

    currSpeed = err * kP + exc_err_total * kI + err_slope * kD;
}

void ExcavationSubsystem::AccelerationControl(double target) {
    accelerationRate = frc::SmartDashboard::GetNumber("Excavation/Acceleration Rate", accelerationRate * 100.0) / 100.0; 
    timer.Start();
    if (timer.AdvanceIfElapsed(units::time::millisecond_t(20))) {
        if (fabs(target - currSpeed) < accelerationRate / 2.0) { // Near Target
            currSpeed = target;
        } else if (target > currSpeed) { // Accelerate Positive
            currSpeed += accelerationRate;
        } else if (target < currSpeed) { // Accelerate Negative
            currSpeed -= accelerationRate;
        }
    }
}