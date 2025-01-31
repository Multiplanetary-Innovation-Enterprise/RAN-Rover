#include "Robot.h"
#include "subsystems/Mobility.h"
#include "subsystems/Excavation.h"
#include "subsystems/Hopper.h"
#include "Constants.h"

#include <frc/XboxController.h>

class Teleop {
    public:
        Teleop(Robot* robot, MobilitySubsystem* mob, ExcavationSubsystem* exc, HopperSubsystem* hop);

        void Init();
        void Periodic();
    private:
        Robot* robot;
        MobilitySubsystem* mob;
        ExcavationSubsystem* exc;
        HopperSubsystem* hop;
        
        frc::XboxController primaryController{PortConstants::primaryController};
        frc::XboxController secondaryController{PortConstants::secondaryController};
        
        void Mobility();
        void Excavation();
        void Hopper();

        double primaryControllerLeftStickDeadzone = 0.1;
        double primaryControllerRightStickDeadzone = 0.1;
        double secondaryControllerLeftStickDeadzone = 0.1;
        double secondaryControllerRightStickDeadzone = 0.1;
};