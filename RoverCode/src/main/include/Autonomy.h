
#include <frc/SPI.h>
#include <frc/Timer.h>
#include <frc2/command/CommandPtr.h>

#include "Robot.h"
#include "Vision.h"
#include "subsystems/Mobility.h"
#include "subsystems/Hopper.h"
#include "subsystems/Excavation.h"

#include "commands/SteerWheel.h"
#include "commands/DistanceDrive.h" 

class Autonomy {
    public:
        enum Phase {
            TRAVERSAL,
            EXC_ORIENT,
            EXC_ACTION,
            DEP_ORIENT,
            DEP_ACTION
        };

        Autonomy(Robot* robot, Vision* vision, MobilitySubsystem* mob, ExcavationSubsystem* exc, HopperSubsystem* hop);

        void Init();
        void Periodic();

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
        Phase currentPhase = TRAVERSAL;
        Robot* robot;
        Vision* vision;
        MobilitySubsystem* mob;
        HopperSubsystem* hop;
        ExcavationSubsystem* exc;
        frc::Timer timer;

        // Autonomous Helper Functions
        bool FindBeacon(int tagId);
        bool NavigateToBeacon(int tagId);

        // Autonomous Commands
        SteerWheel steer{mob};
        DistanceDrive distDrive{mob};

        //////////////////////////
        // Autonomous Variables //
        //////////////////////////

        // Dep Action
        units::time::second_t hopperEmptyTime{6.0}; // In Seconds
        double hopperEmptySpeed = 0.75; // Motor Percentage
};