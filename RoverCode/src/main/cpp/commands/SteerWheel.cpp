#pragma once

#include "commands/SteerWheel.h"

SteerWheel::SteerWheel(MobilitySubsystem* mob) : mob(mob) {
    AddRequirements(mob);
}

void SteerWheel::Initialize() {
    for (int i = 0; i < 4; i++) {
        MobilitySubsystem::Wheel w = MobilitySubsystem::Wheel(i);
        if (mob->GetAngle(w) > angleTarget[w] + angleMargin[w]) {
            dir[w] = false;
            mob->StartActuate(w, false);
            wpi::outs() << "Moving Left\n";
        } else if (mob->GetAngle(w) < angleTarget[w] - angleMargin[w]) {
            dir[w] = true;
            mob->StartActuate(w, true);
            wpi::outs() << "Moving Right\n";
        }
    }
}

void SteerWheel::Execute() {
    for (int i = 0; i < 4; i++) {
        MobilitySubsystem::Wheel w = MobilitySubsystem::Wheel(i);
        if (!dir[w] && mob->GetAngle(w) < angleTarget[w] - angleMargin[w]) {
            dir[w] = true;
            mob->StartActuate(w, true);
            wpi::outs() << "Moving Right, Changed Direction\n";
        } else if (dir[w] && mob->GetAngle(w) > angleTarget[w] + angleMargin[w]) {
            dir[w] = false;
            mob->StartActuate(w, false);
            wpi::outs() << "Moving Left, Changing Direction\n";
        }
    }
}

bool SteerWheel::IsFinished() {
    bool finished = false;
    for (int i = 0; i < 4; i++) { 
        MobilitySubsystem::Wheel w = MobilitySubsystem::Wheel(i);
        finished &= mob->GetAngle(w) >= angleTarget[w] - angleMargin[w]
                 && mob->GetAngle(w) <= angleTarget[w] + angleMargin[w];
    }
    return finished;
}

void SteerWheel::End(bool interrupted) {
    for (int i = 0; i < 4; i++)
        mob->StopActuate(MobilitySubsystem::Wheel(i), false);
    wpi::outs() << "Stopped Steering\n";
}

// Speed Range 0.7 to 0.9 cm/s depending on load
// Max Range of front wheel (-28, 28), back wheel (-28 to 58)
// Expect 3.3s from 0 to 28 degrees at full load
//        8.2s from 0 to 58 degrees at full load

void SteerWheel::Ackermann(double angle) {
    // Will need calculations from MAD for how to calculate this.
    angleTarget[MobilitySubsystem::Wheel::FRONT_LEFT] = angle;
    angleTarget[MobilitySubsystem::Wheel::FRONT_RIGHT] = -angle;
    angleTarget[MobilitySubsystem::Wheel::BACK_LEFT] = -angle;
    angleTarget[MobilitySubsystem::Wheel::BACK_RIGHT] = angle;
}

void SteerWheel::Crab(double angle) {
    angleTarget[MobilitySubsystem::Wheel::FRONT_LEFT] = angle;
    angleTarget[MobilitySubsystem::Wheel::FRONT_RIGHT] = -angle;
    angleTarget[MobilitySubsystem::Wheel::BACK_LEFT] = angle;
    angleTarget[MobilitySubsystem::Wheel::BACK_RIGHT] = -angle;
}

void SteerWheel::ZeroPoint() {
    angleTarget[MobilitySubsystem::Wheel::FRONT_LEFT] = 28.0;
    angleTarget[MobilitySubsystem::Wheel::FRONT_RIGHT] = 28.0;
    angleTarget[MobilitySubsystem::Wheel::BACK_LEFT] = -58.0;
    angleTarget[MobilitySubsystem::Wheel::BACK_RIGHT] = -58.0;
}

void SteerWheel::Straight() {
    angleTarget[MobilitySubsystem::Wheel::FRONT_LEFT] = 0.0;
    angleTarget[MobilitySubsystem::Wheel::FRONT_RIGHT] = 0.0;
    angleTarget[MobilitySubsystem::Wheel::BACK_LEFT] = 0.0;
    angleTarget[MobilitySubsystem::Wheel::BACK_RIGHT] = 0.0;
}