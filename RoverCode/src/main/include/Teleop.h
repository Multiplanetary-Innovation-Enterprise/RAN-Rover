#pragma once
#define _USE_MATH_DEFINES

#include "subsystems/Mobility.h"
#include "subsystems/Excavation.h"
#include "subsystems/Hopper.h"
#include "subsystems/Blinkin.h"
#include "Constants.h"

#include <frc/XboxController.h>

class Teleop {
    public:

        enum MobilityMode {
            CRAWL = 0,
            ACKERMANN = 1,
            FREE = 2,
            ZERO_POINT = 3,
            CRAB = 4
        };

        Teleop();
        void SetSystems(MobilitySubsystem* mob, ExcavationSubsystem* exc, HopperSubsystem* hop);

        void Init();
        void Periodic();
    private:
        MobilitySubsystem* mob;
        ExcavationSubsystem* exc;
        HopperSubsystem* hop;
        
        frc::XboxController primaryController{PortConstants::primaryController};
        frc::XboxController secondaryController{PortConstants::secondaryController};

        void Mobility();
        void Excavation();
        void Hopper();

        MobilityMode mobMode = FREE;

        double primaryControllerLeftStickDeadzone = 0.1;
        double primaryControllerRightStickDeadzone = 0.1;
        double secondaryControllerLeftStickDeadzone = 0.1;
        double secondaryControllerRightStickDeadzone = 0.1;
};