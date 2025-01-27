
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
        IMUSendable imu{frc::SPI::Port::kMXP};
        frc::Timer timerA, timerB;

        // Autonomous Helper Functions
        bool FindBeacon(int tagId);
        bool NavigateToBeacon(int tagId);
        bool ZeroPointTurn(double degrees);

        // Autonomous Commands
        SteerWheel steer{mob};
        DistanceDrive distDrive{mob};

        //////////////////////////
        // Autonomous Variables //
        //////////////////////////

        // Mobility
        double zeroPointDriveSpeed = 0.3;

        // Exc Action
        double excavationSpinSpeed = 0.5;
        double excavationSpinStartAngle = 25.0;
        double excavationStowAngle = -10.0;
        units::time::second_t excavationCycleTime{10.0};
        double hopperIndexSpeed = 0.5;
        units::time::second_t hopperIndexTime{1.0};
        units::time::second_t hopperIndexPeriod{5.0};

        // Dep Action
        double hopperEmptySpeed = 0.75; // Motor Percentage
        units::time::second_t hopperEmptyTime{6.0}; // In Seconds
};