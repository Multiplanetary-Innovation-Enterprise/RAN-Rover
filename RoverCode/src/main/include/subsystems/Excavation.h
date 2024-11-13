#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <wpi/raw_ostream.h>
#include <frc/DigitalOutput.h>
#include <frc/AnalogPotentiometer.h>
#include "Constants.h"
#include "sendables/MotorSendable.h"

class ExcavationSubsystem : public frc2::SubsystemBase {
    public:
        ExcavationSubsystem();
        void Periodic() override;
        void SimulationPeriodic() override;
        void Reset();

        bool canSpin();

        void HoldLock(bool hold);
        void Spin(double speed, bool invert);
        void Stop();

        void StartActuate(bool dir);
        void StopActuate(bool limitHit);

    private:
        double maxSpinSpeed = 0.25; // The max speed to spin the excavator.
        double actuateSpeed = 0.025; // The speed to actuate.
        double actuateSpinThreshold = 0.2; // When during the actuation process is spin able to occur.
        double actuateMin = 0.05; // Minimum limit of potentiometer for actuation.
        double actuateMax = 0.38; // Maximum limit of potentiometer for actuation.
        
        bool isSpinning;
        int actuatingDir;
        bool isLocked;
        MotorSendable excSpin{PortConstants::excSpin, rev::CANSparkMax::MotorType::kBrushless};

        frc::DigitalOutput excLeftActVel{PortConstants::excLeftActVel};
        frc::DigitalOutput excLeftActDir{PortConstants::excLeftActDir};
        frc::AnalogPotentiometer excLeftPot{PortConstants::excLeftPot, 1.0, 0.0};

        frc::DigitalOutput excRightActVel{PortConstants::excRightActVel};
        frc::DigitalOutput excRightActDir{PortConstants::excRightActDir};
        frc::AnalogPotentiometer excRightPot{PortConstants::excRightPot, 1.0, 0.0};
};