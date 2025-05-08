#pragma once
#define _USE_MATH_DEFINES

#include <frc/SPI.h>
#include <frc/Timer.h>
#include <frc2/command/CommandPtr.h>
#include <vector>

#include "Vision.h"
#include "subsystems/Mobility.h"
#include "subsystems/Hopper.h"
#include "subsystems/Excavation.h"
#include "subsystems/Blinkin.h"
#include "sendables/Coord.h"
#include "Localization.h"

class Autonomy {
    public:
        enum Phase {
            TRAVERSAL,
            EXC_ORIENT,
            EXC_ACTION,
            DEP_ORIENT,
            DEP_ACTION
        };

        Autonomy();
        void SetSystems(Vision* vision, Localization* localization, MobilitySubsystem* mob, ExcavationSubsystem* exc, HopperSubsystem* hop);

        void Init();
        bool Periodic();

        void TraversalInit();
        bool TraversalPeriodic();
        void ExcOrientInit();
        bool ExcOrientPeriodic();
        void ExcActionInit();
        bool ExcActionPeriodic();
        void DepOrientInit();
        bool DepOrientPeriodic();
        void DepActionInit();
        bool DepActionPeriodic();
    private:
        Vision* vision;
        Localization* localization;
        MobilitySubsystem* mob;
        HopperSubsystem* hop;
        ExcavationSubsystem* exc;

        frc::Timer timer;
        frc::Timer crawlTimer;
        Phase currentPhase = TRAVERSAL;

        // Autonomous Helper Functions
        void Kill();
        int calcTrenchCount();
        double calcTrenchAngle();

        //////////////////////////
        // Autonomous Variables //
        //////////////////////////

        // Traversal
        Coord targetPos{5.38, 1.35};
        double driveTimePerCell = 1.0;
        std::array<units::time::second_t, 4> turnTimes{units::time::second_t{1.0}, units::time::second_t{2.0}, units::time::second_t{3.0}, units::time::second_t{4.0}};

        // Exc Orient
        int trenchIndex = -1;
        double trenchGap = 0.5;
        double trenchStartDist = 1.0;
        units::time::second_t failsafeTime{8.0};
        
        // Exc Action
        units::time::second_t excavationSpinTime{10.0};

        // Dep Orient
        double crawlTimeToMaxTimeFactor = 2.0;
        // Uses most of Exc Orient values, as it's just reversed.

        // Dep Action
        units::time::second_t hopperSpinTime{14.0}; // In Seconds
};