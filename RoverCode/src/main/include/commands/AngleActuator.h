#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/DigitalOutput.h>
#include <frc/AnalogPotentiometer.h>
#include "subsystems/Excavation.h"

class AngleActuator : public frc2::CommandHelper<frc2::Command, AngleActuator> {
    public:
        explicit AngleActuator(ExcavationSubsystem* exc);
        void Initialize() override;
        void Execute() override;
        bool IsFinished() override;
        void End(bool interrupted) override;
        
        /**
         * Actuator Index:
         * 0 - Front Left
         * 1 - Back Left
         * 2 - Front Right
         * 3 - Back Right
         * 
         * Angle in Degrees
        */
        void Set(int actuatorIndex, double angle);
    private:
        ExcavationSubsystem* exc;

        int actIndex = 0;
        double angleTarget = 0.0;
        double angleMargin = 1.0;
        bool dir = false;
};