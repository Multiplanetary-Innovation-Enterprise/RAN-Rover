#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <wpi/raw_ostream.h>
#include "Constants.h"
#include "sendables/MotorSendable.h"

class HopperSubsystem : public frc2::SubsystemBase {
    public:        
        HopperSubsystem();
        void Periodic() override;
        void SimulationPeriodic() override;
        void Reset();

        void HoldLock(bool hold);
        void Spin(double speed, bool invert);
        void Stop();
    private:
        double maxSpinSpeed = 0.15;
        bool isSpinning;
        bool isLocked;
        MotorSendable hopSpin{PortConstants::hopSpin, rev::CANSparkMax::MotorType::kBrushless};
};