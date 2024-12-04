#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Mobility.h"

class DistanceDrive : public frc2::CommandHelper<frc2::Command, DistanceDrive> {
    public:
        explicit DistanceDrive(MobilitySubsystem* mob);
        void Initialize() override;
        void Execute() override;
        bool IsFinished() override;
        void End(bool interrupted) override;

        void Set(double dist);
    private:
        MobilitySubsystem* mob;

        double dist = 0.0;
        double distSlow = 40.0;
        double distLimit = 25.0;
};