#pragma once

#include "commands/DistanceDrive.h"
#include <frc/smartdashboard/SmartDashboard.h>

DistanceDrive::DistanceDrive(MobilitySubsystem* mob) : mob(mob) {
    AddRequirements(mob);

    frc::SmartDashboard::PutNumber("Distance Drive Slow", distSlow);
    frc::SmartDashboard::PutNumber("Distance Drive Stop", distLimit);
}

void DistanceDrive::Initialize() {

    distSlow = frc::SmartDashboard::GetNumber("Distance Drive Slow", distSlow);
    distLimit = frc::SmartDashboard::GetNumber("Distnace Drive Stop", distLimit);

    if (dist > distSlow) {
        mob->Drive({0.4, 0.4, 0.4, 0.4});
        wpi::outs() << "Drive Fast\n";
    } else if (dist > distLimit) {
        mob->Drive({0.2, 0.2, 0.2, 0.2});
        wpi::outs() << "Drive Slow\n";
    }
}

void DistanceDrive::Execute() {
    if (dist > distSlow) {
        mob->Drive({0.4, 0.4, 0.4, 0.4});
        wpi::outs() << "Drive Fast, Updated\n";
    } else if (dist > distLimit) {
        mob->Drive({0.2, 0.2, 0.2, 0.2});
        wpi::outs() << "Drive Slow, Updated\n";
    }
}

bool DistanceDrive::IsFinished() {
    return dist <= distLimit;
}

void DistanceDrive::End(bool interrupted) {
    mob->StopAll();
    wpi::outs() << "Stopped Driving\n";
}

void DistanceDrive::Set(double d) {
    dist = d;
}