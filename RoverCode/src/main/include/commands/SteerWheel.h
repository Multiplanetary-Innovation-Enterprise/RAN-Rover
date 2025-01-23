#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/DigitalOutput.h>
#include <frc/AnalogPotentiometer.h>
#include "subsystems/Mobility.h"

class SteerWheel : public frc2::CommandHelper<frc2::Command, SteerWheel> {
    public:
        explicit SteerWheel(MobilitySubsystem* mob);
        void Initialize() override;
        void Execute() override;
        bool IsFinished() override;
        void End(bool interrupted) override;

        void Ackermann(double angle);
        void Crab(double angle);
        void ZeroPoint();
        void Straight();
    private:
        MobilitySubsystem* mob;

        double angleTarget[4] = {0.0, 0.0, 0.0, 0.0};
        double angleMargin[4] = {1.0, 1.0, 1.0, 1.0};
        bool dir[4] = {false, false, false, false};
};