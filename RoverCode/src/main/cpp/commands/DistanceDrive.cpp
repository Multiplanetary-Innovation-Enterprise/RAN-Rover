#pragma once

#include "commands/DistanceDrive.h"

DistanceDrive::DistanceDrive(MobilitySubsystem* mob) : mob(mob) {
    AddRequirements(mob);
}

void DistanceDrive::Initialize() {
    if (dist > distSlow) {
        mob->Drive(0.4, 0.4, false);
        wpi::outs() << "Drive Fast\n";
    } else if (dist > distLimit) {
        mob->Drive(0.2, 0.2, false);
        wpi::outs() << "Drive Slow\n";
    }
}

void DistanceDrive::Execute() {
    if (dist > distSlow) {
        mob->Drive(0.4, 0.4, false);
        wpi::outs() << "Drive Fast, Updated\n";
    } else if (dist > distLimit) {
        mob->Drive(0.2, 0.2, false);
        wpi::outs() << "Drive Slow, Updated\n";
    }
}

bool DistanceDrive::IsFinished() {
    return dist <= distLimit;
}

void DistanceDrive::End(bool interrupted) {
    mob->Stop();
    wpi::outs() << "Stopped Driving\n";
}

void DistanceDrive::Set(double dist) {
    dist = dist;
}