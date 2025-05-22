#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <wpi/raw_ostream.h>
#include <frc/DutyCycle.h>
#include <frc/DigitalOutput.h>
#include <frc/AnalogPotentiometer.h>
#include "Constants.h"
#include <frc/Timer.h>
#include "sendables/MotorSendable.h"

class ExcavationSubsystem : public frc2::SubsystemBase {
    public:
        ExcavationSubsystem();
        void Periodic() override;
        void SimulationPeriodic() override;
        void Reset();

        bool shouldActuateSlow();

        void HoldLock(bool hold);
        void Spin(double speed, bool invert);
        void Stop();

        void StartActuate(bool dir);
        void StopActuate();
        double GetActuateUpper();
        double GetActuateLower();
        double GetSpeed();

        double actuateMin = 0.0; // Minimum limit of potentiometer for actuation.
        double actuateMax = 3.94; // Maximum limit of potentiometer for actuation.

    private:
        void PseudoPID(double targetVelocity);
        void AccelerationControl(double target);
        double steadySpinSpeed = 0.625; // The max speed to spin the excavator.
        double plungeSpinSpeed = 0.625;
        double actuateFastSpeed = 1.0; // The speed to actuate.
        double actuateSlowSpeed = 0.1;
        double actuatePW = 200.0;
        double actuateSpinThreshold = 3.4; // When during the actuation process is spin able to occur.
        double accelerationRate = 1.0;

        // PID coefficients
        double kP = 0.00001, kI = 0.000025, kD = 0.0;

        double actPotDiff = 0.43;
        double currSpeed = 0.0;
        bool isSpinning;
        int actuatingDir;
        bool isLocked;
        frc::Timer timer;
        MotorSendable excSpin{PortConstants::excSpin, rev::CANSparkMax::MotorType::kBrushless};

        frc::DigitalOutput excLeftActVel{PortConstants::excLeftActVel};
        frc::DigitalOutput excLeftActDir{PortConstants::excLeftActDir};
        frc::AnalogPotentiometer excLeftPot{PortConstants::excLeftPot, 10.0, 0.0};

        frc::DigitalOutput excRightActVel{PortConstants::excRightActVel};
        frc::DigitalOutput excRightActDir{PortConstants::excRightActDir};
        frc::DutyCycle excRightActPWM{excRightActVel};
        frc::AnalogPotentiometer excRightPot{PortConstants::excRightPot, 10.0, 0.0};
};