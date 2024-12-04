#pragma once

#include "commands/AngleActuator.h"

AngleActuator::AngleActuator(ExcavationSubsystem* exc) : exc(exc) {
    AddRequirements(exc);
}

void AngleActuator::Initialize() {
    if (exc->ActuateAngle() > angleTarget + angleMargin) {
        dir = false;
        exc->StartActuate(false);
        wpi::outs() << "Moving Left\n";
    } else if (exc->ActuateAngle() < angleTarget - angleMargin) {
        dir = true;
        exc->StartActuate(true);
        wpi::outs() << "Moving Right\n";
    }
}

void AngleActuator::Execute() {
    if (!dir && exc->ActuateAngle() < angleTarget - angleMargin) {
        dir = true;
        exc->StartActuate(true);
        wpi::outs() << "Moving Right, Changed Direction\n";
    } else if (dir && exc->ActuateAngle() > angleTarget + angleMargin) {
        dir = false;
        exc->StartActuate(false);
        wpi::outs() << "Moving Left, Changing Direction\n";
    }
}

bool AngleActuator::IsFinished() {
    return (!dir && exc->ActuateAngle() <= angleTarget + angleMargin)
        || (dir && exc->ActuateAngle() >= angleTarget - angleMargin);
}

void AngleActuator::End(bool interrupted) {
    exc->StopActuate(false);
    wpi::outs() << "Stopped Moving\n";
}

void AngleActuator::Set(int actuatorIndex, double angle) {
    actIndex = actuatorIndex;
    angleTarget = angle;
}