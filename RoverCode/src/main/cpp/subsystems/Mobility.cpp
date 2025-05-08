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

    frc::SmartDashboard::PutNumber("Mobility/Drive Max Speed", maxDriveSpeed);
    frc::SmartDashboard::SetPersistent("Mobility/Drive Max Speed");
    frc::SmartDashboard::PutNumber("Mobility/Crawl Max Speed", maxCrawlSpeed);
    frc::SmartDashboard::SetPersistent("Mobility/Crawl Max Speed");

    frc::SmartDashboard::PutNumber("Mobility/Steer Margin", steerMargin);
    frc::SmartDashboard::SetPersistent("Mobility/Steer Margin");

    frc::SmartDashboard::PutNumber("Mobility/Acceleration Rate", accelerationRate);
    frc::SmartDashboard::SetPersistent("Mobility/Acceleration Rate");
    frc::SmartDashboard::PutBoolean("Mobility/Use SlipControl", useSlipControl);
    frc::SmartDashboard::SetPersistent("Mobility/Use SlipControl");

    frc::SmartDashboard::PutNumber("Mobility/Slip/Current Threshold", currentThreshold);
    frc::SmartDashboard::SetPersistent("Mobility/Slip/Current Threshold");
    frc::SmartDashboard::PutNumber("Mobility/Slip/Off Cycles", cyclesSlipOff);
    frc::SmartDashboard::SetPersistent("Mobility/Slip/Off Cycles");
    frc::SmartDashboard::PutNumber("Mobility/Slip/Try Cycles", cyclesSlipTry);
    frc::SmartDashboard::SetPersistent("Mobility/Slip/Try Cycles");

    for (int i = 0; i < 4; i++) {
        frc::SmartDashboard::PutNumber("Mobility/" + std::array<std::string, 4>{"FrontLeft", "FrontRight", "BackLeft", "BackRight"}[i] + "/Steer Min", actuateLimits[i][0]);
        frc::SmartDashboard::SetPersistent("Mobility/" + std::array<std::string, 4>{"FrontLeft", "FrontRight", "BackLeft", "BackRight"}[i] + "/Steer Min");
        frc::SmartDashboard::PutNumber("Mobility/" + std::array<std::string, 4>{"FrontLeft", "FrontRight", "BackLeft", "BackRight"}[i] + "/Steer Max", actuateLimits[i][1]);
        frc::SmartDashboard::SetPersistent("Mobility/" + std::array<std::string, 4>{"FrontLeft", "FrontRight", "BackLeft", "BackRight"}[i] + "/Steer Max");
        frc::SmartDashboard::PutData("Mobility/" + std::array<std::string, 4>{"FrontLeft", "FrontRight", "BackLeft", "BackRight"}[i] + "/Motor", &motor[i]);
        frc::SmartDashboard::PutData("Mobility/" + std::array<std::string, 4>{"FrontLeft", "FrontRight", "BackLeft", "BackRight"}[i] + "/Pot", &actPot[i]);
        frc::SmartDashboard::PutBoolean("Mobility/" + std::array<std::string, 4>{"FrontLeft", "FrontRight", "BackLeft", "BackRight"}[i] + "/Steer Done", false);
    }
    frc::SmartDashboard::PutNumber("Mobility/PIDControllers/P Gain", kP);
    frc::SmartDashboard::SetPersistent("Mobility/PIDControllers/P Gain");
    frc::SmartDashboard::PutNumber("Mobility/PIDControllers/I Gain", kI);
    frc::SmartDashboard::SetPersistent("Mobility/PIDControllers/I Gain");
    frc::SmartDashboard::PutNumber("Mobility/PIDControllers/D Gain", kD);
    frc::SmartDashboard::SetPersistent("Mobility/PIDControllers/D Gain");

    Reset();
}

void MobilitySubsystem::Periodic() {

    for (int i = 0; i < 4; i++) {
        Wheel w = Wheel(i);
        frc::SmartDashboard::PutNumber("Mobility/" + std::array<std::string, 4>{"FrontLeft", "FrontRight", "BackLeft", "BackRight"}[w] + "/Steer Angle", GetAngle(w));
        if (actuateDirs[w] == 1) {
            if (GetAngle(w) <= actuateLimits[w][0] - 1) {
                StopActuate(w);
            }
        } else if (actuateDirs[w] == -1) {
            if (GetAngle(w) >= actuateLimits[w][1] + 1) {
                StopActuate(w);
            }
        }
    }
}
void MobilitySubsystem::SimulationPeriodic() {}

void MobilitySubsystem::Reset() {
    
    motor[FRONT_LEFT].StopMotor();
    motor[FRONT_RIGHT].StopMotor();
    motor[BACK_LEFT].StopMotor();
    motor[BACK_RIGHT].StopMotor();
    currSpeed[FRONT_LEFT] = 0.0;
    currSpeed[FRONT_RIGHT] = 0.0;
    currSpeed[BACK_LEFT] = 0.0;
    currSpeed[BACK_RIGHT] = 0.0;

    StopActuate(FRONT_LEFT);
    StopActuate(FRONT_RIGHT);
    StopActuate(BACK_LEFT);
    StopActuate(BACK_RIGHT);

    isSpinning = false;
    leftCrawl = false;
    rightCrawl = false;
}

std::array<double, 4> MobilitySubsystem::SlipControl(std::array<double, 4> in) {
    std::array<double, 4> out = in;
    useSlipControl = frc::SmartDashboard::GetBoolean("Mobility/Use SlipControl", useSlipControl);
    if (!useSlipControl)
        return out;

    currentThreshold = frc::SmartDashboard::GetNumber("Mobility/Slip/Current Threshold", currentThreshold);
    cyclesSlipOff = frc::SmartDashboard::GetNumber("Mobility/Slip/Off Cycles", cyclesSlipOff);
    cyclesSlipTry = frc::SmartDashboard::GetNumber("Mobility/Slip/Try Cycles", cyclesSlipTry);

    for (int i = 0; i < 4; i++) {
        if (fabs(in[i]) > 0.0) { // Do we want this motor to move?
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

void MobilitySubsystem::AccelerationControl(std::array<double, 4> target) {
    accelerationRate = (frc::SmartDashboard::GetNumber("Mobility/Acceleration Rate", accelerationRate * 100.0) / 100.0) / maxDriveSpeed; 
    timer.Start();
    if (timer.AdvanceIfElapsed(units::time::millisecond_t(20))) {
        for (int i = 0; i < 4; i++) {
            if (fabs(target[i] - currSpeed[i]) < accelerationRate / 2.0) { // Near Target
                currSpeed[i] = target[i];
            } else if (target[i] > currSpeed[i]) { // Accelerate Positive
                currSpeed[i] += accelerationRate;
            } else if (target[i] < currSpeed[i]) { // Accelerate Negative
                currSpeed[i] -= accelerationRate;
            }
        }
    }
}

std::array<double, 4> err_total = {0.0, 0.0, 0.0, 0.0};
std::array<double, 4> err_prev = {0.0, 0.0, 0.0, 0.0};
std::array<double, 4> prev_time = {0.0, 0.0, 0.0, 0.0};

void MobilitySubsystem::PseudoPID(std::array<double, 4> targetVelocity) {
    // read PID coefficients from SmartDashboard
    double kP = frc::SmartDashboard::GetNumber("Mobility/PIDControllers/P Gain", 0);
    double kI = frc::SmartDashboard::GetNumber("Mobility/PIDControllers/I Gain", 0);
    double kD = frc::SmartDashboard::GetNumber("Mobility/PIDControllers/D Gain", 0);
    
    timer.Start();
    for (int i = 0; i < 4; i++) {
        double curr_time = timer.Get().value();
        double err = targetVelocity[i] - motor[i].encoder.GetVelocity();
        err_total[i]= err_total[i] + err;
        double err_slope = (err - err_prev[i]) / (curr_time  - prev_time[i]);

        prev_time[i] = curr_time;
        err_prev[i] = err;

        currSpeed[i] = err * kP + err_total[i] * kI + err_slope * kD;
    }
}

void MobilitySubsystem::Drive(std::array<double, 4> speed) {
    maxDriveSpeed = frc::SmartDashboard::GetNumber("Mobility/Drive Max Speed", maxDriveSpeed);

    isSpinning = true;
    wpi::outs() << "Drive Max Speed: " << std::to_string(int(maxDriveSpeed * 100.0)) << "% | Curr, Target\n";
    
    std::array<double, 4> controlledSpeed = SlipControl(speed);
    AccelerationControl(controlledSpeed);

    for (int i = 0; i < 4; i++) {
        wpi::outs() << std::array<std::string, 4>{"\tFrontLeft: ", "\tFrontRight: ", "\tBackLeft: ", "\tBackRight: "}[i] << std::to_string(int(currSpeed[i] * 100.0)) << "%, " << std::to_string(int(controlledSpeed[i] * 100.0)) << "%\n";
        motor[i].Set(currSpeed[i] * maxDriveSpeed);
    }
}

void MobilitySubsystem::Crawl(bool forward) {
    maxCrawlSpeed = frc::SmartDashboard::GetNumber("Mobility/Crawl Max Speed", maxCrawlSpeed);

    std::array<double, 4> targetSpeed = {maxCrawlSpeed, maxCrawlSpeed, maxCrawlSpeed, maxCrawlSpeed};
    if (!forward)
        targetSpeed = {-maxCrawlSpeed, -maxCrawlSpeed, -maxCrawlSpeed, -maxCrawlSpeed};

    isSpinning = true;
    PseudoPID(targetSpeed);

    wpi::outs() << "Crawl <Output, Target, Actual>: < ";
    for (int i = 0; i < 4; i++) {
        wpi::outs() << "(" << std::to_string(int(currSpeed[i] * 100.0)) << "%, " << std::to_string(int(targetSpeed[i])) << " RPM, " << std::to_string(int(motor[i].encoder.GetVelocity())) << " RPM)" << (i < 4 ? ", " : " >\n");
        motor[i].Set(currSpeed[i]);
        // Gearboxes 125:1
        // Max 5600 RPM
        // Radius of wheels 0.175 meters
    }
}

void MobilitySubsystem::StopAll() {
    if (!isSpinning) return;
    wpi::outs() << "Stop Motors\n";
    isSpinning = false;
    currSpeed[FRONT_LEFT] = 0.0;
    currSpeed[FRONT_RIGHT] = 0.0;
    currSpeed[BACK_LEFT] = 0.0;
    currSpeed[BACK_RIGHT] = 0.0;
    motor[FRONT_LEFT].StopMotor();
    motor[FRONT_RIGHT].StopMotor();
    motor[BACK_LEFT].StopMotor();
    motor[BACK_RIGHT].StopMotor();
}

void MobilitySubsystem::StartActuate(Wheel w, bool dir) {

    actuateLimits[w][0] = frc::SmartDashboard::GetNumber("Mobility/" + std::array<std::string, 4>{"FrontLeft", "FrontRight", "BackLeft", "BackRight"}[w] + "/Steer Min", actuateLimits[w][0]);
    actuateLimits[w][1] = frc::SmartDashboard::GetNumber("Mobility/" + std::array<std::string, 4>{"FrontLeft", "FrontRight", "BackLeft", "BackRight"}[w] + "/Steer Max", actuateLimits[w][1]);
    steerMargin = frc::SmartDashboard::GetNumber("Mobility/Steer Margin", steerMargin);

    if (dir) {
        if (GetAngle(w) <= actuateLimits[w][0]) {
            StopActuate(w);
            return;
        } else if (actuateDirs[w] != 1) {
            actuateDirs[w] = 1;
        } else {
            return;
        }
    } else {
        if (GetAngle(w) >= actuateLimits[w][1]) {
            StopActuate(w);
            return;
        } else if (actuateDirs[w] != -1) {
            actuateDirs[w] = -1;
        } else {
            return;
        }
    }

    wpi::outs() << "StartActuate " << (std::array<std::string, 4>{"FrontLeft", "FrontRight", "BackLeft", "BackRight"}[w]) << (dir ? " Inward" : " Outward") << "\n";
    frc::SmartDashboard::PutNumber("Mobility/" + std::array<std::string, 4>{"FrontLeft", "FrontRight", "BackLeft", "BackRight"}[w] + "/Steer Direction", dir ? 1 : -1);
    actDir[w].Set(dir);
    actVel[w].Set(true);
}

void MobilitySubsystem::StopActuate(Wheel w) {
    if (actuateDirs[w] == 0)
        return;
    actuateDirs[w] = 0;
    wpi::outs() << "StopActuate " << (std::array<std::string, 4>{"FrontLeft", "FrontRight", "BackLeft", "BackRight"}[w]) << "\n";
    frc::SmartDashboard::PutNumber("Mobility/" + std::array<std::string, 4>{"FrontLeft", "FrontRight", "BackLeft", "BackRight"}[w] + "/Steer Direction", 0);
    actVel[w].Set(false);
    actDir[w].Set(false);
}

bool MobilitySubsystem::SteerTo(std::array<double, 4> target) {
    bool allFinished = true;
    for (int i = 0; i < 4; i++) {
        Wheel w = Wheel(i);
        double a = 0.0;
        if (target[w] != 0.0)
            a = std::max(actuateLimits[w][0], std::min(fabs(target[w]), actuateLimits[w][1])) * (target[w] / fabs(target[w]));
        if (GetAngle(w) >= a - steerMargin && GetAngle(w) <= a + steerMargin) { // In target range
            frc::SmartDashboard::PutBoolean("Mobility/" + std::array<std::string, 4>{"FrontLeft", "FrontRight", "BackLeft", "BackRight"}[w] + "/Steer Done", true);
            if (GetAngle(w) >= a - (steerMargin / 2.0) && GetAngle(w) <= a + (steerMargin / 2.0))
                StopActuate(w);
        } else if (GetAngle(w) < a - steerMargin) {
            frc::SmartDashboard::PutBoolean("Mobility/" + std::array<std::string, 4>{"FrontLeft", "FrontRight", "BackLeft", "BackRight"}[w] + "/Steer Done", false);
            StartActuate(w, false);
            allFinished = false;
        } else if (GetAngle(w) > a + steerMargin) {
            frc::SmartDashboard::PutBoolean("Mobility/" + std::array<std::string, 4>{"FrontLeft", "FrontRight", "BackLeft", "BackRight"}[w] + "/Steer Done", false);
            StartActuate(w, true);
            allFinished = false;
        }
    }
    return allFinished;
}

double MobilitySubsystem::GetAngle(Wheel w) {

    double pot = actPot[w].Get();
    double len = pot; 
    double ang = 0;

    if (w == FRONT_LEFT || w == FRONT_RIGHT) {
        len = (len / 0.50) * 2.0; // Potentiometer max * Max Extension Length
        // Angle (Radians) to Length (Inches)
        // Length = sqrt(78.6639 in - 34.5633 in * cos(Angle + 1.4027 radians)) - 7.51 in

        // Length (Inches) to Angle (Radians)
        // Angle = acos((pow(Length + 7.51 in, 2) - 78.6638 in) / -34.5633 in) - 1.4027 radians
        ang = (acos((pow(len + 7.51, 2) - 78.6638) / -34.5633) - 1.4027) * 180.0 / M_PI;
    } else if (w == BACK_LEFT || w == BACK_RIGHT) {
        len = (len / 0.99) * 4.0; // Potentiometer max * Max Extension Length
        // Angle (Radians) to Length (Inches)
        // Length = sqrt(135.492 in - 55.9758 in * cos(Angle + 1.1353 radians)) - 9.51 in

        // Length (Inches) to Angle (Radians)
        // Angle = acos((pow(Length + 9.51 in, 2) - 135.492 in) / -55.9758 in) - 1.1353 radians
        ang = (acos((pow(len + 9.51, 2) - 135.492) / -55.9758) - 1.1353) * 180.0 / M_PI;
    }

    return ang;
}