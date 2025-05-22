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
#include "Pathfinding.h"

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
        Pathfinding* pathing;
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
        double driveTimePerCell = 0.04;
        std::array<units::time::second_t, 4> turnTimes{units::time::second_t{1.0}, units::time::second_t{3.1}, units::time::second_t{3.0}, units::time::second_t{4.0}};
        double fromWallDistance = 0.25;
        double pathStartDistance = 2.0;
        double pathDistance = 3.0;
        double toBermDistance = -1;
        int startingFacingDirection = 3; // 0 is North, 1 is East, 2 is South, 3 is West

        // Exc Orient
        int trenchIndex = 2;
        double trenchGap = 0.1;
        double trenchStartDist = 0.5;
        units::time::second_t failsafeTime{1.5};
        Node* excValidationNode = new Node(0.0, 0.0, 0.0, 0.0, 0.0, MobilitySubsystem::MobilityMode::FREE, 0.0, false);
        
        // Exc Action
        double startExcavationSpin = 3.1;
        units::time::second_t excavationSpinTime{30.0};

        // Dep Orient
        double crawlTimeToMaxTimeFactor = 0.03;
        // Uses most of Exc Orient values, as it's just reversed.

        // Dep Action
        units::time::second_t hopperSpinTime{14.0}; // In Seconds
};