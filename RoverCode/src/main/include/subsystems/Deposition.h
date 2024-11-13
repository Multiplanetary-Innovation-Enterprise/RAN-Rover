#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <wpi/raw_ostream.h>
#include <frc/DigitalOutput.h>
#include <frc/AnalogPotentiometer.h>
#include "Constants.h"
#include "sendables/MotorSendable.h"

class DepositionSubsystem : public frc2::SubsystemBase {
    public:
        DepositionSubsystem();
        void Periodic() override;
        void SimulationPeriodic() override;
        void Reset();

        bool canSpin();

        void Spin(double speed, bool invert);
        void Stop();

        void StartActuate(bool dir);
        void StopActuate(bool limitHit);

    private:
        double maxSpinSpeed = 0.25;
        double actuateSpeed = 0.025; // The speed to actuate.
        double actuateSpinThreshold = 0.15; // When during the actuation process is spin able to occur.
        double actuateMin = 0.1; // Minimum limit of potentiometer for actuation.
        double actuateMax = 0.35; // Maximum limit of potentiometer for actuation.
        
        bool isSpinning;
        int actuatingDir;
        MotorSendable depSpin{PortConstants::depSpin, rev::CANSparkMax::MotorType::kBrushless};

        frc::DigitalOutput depLeftActVel{PortConstants::depLeftActVel};
        frc::DigitalOutput depLeftActDir{PortConstants::depLeftActDir};
        frc::AnalogPotentiometer depLeftPot{PortConstants::depLeftPot, 1.0, 0.0};

        frc::DigitalOutput depRightActVel{PortConstants::depRightActVel};
        frc::DigitalOutput depRightActDir{PortConstants::depRightActDir};
        frc::AnalogPotentiometer depRightPot{PortConstants::depRightPot, 1.0, 0.0};
};