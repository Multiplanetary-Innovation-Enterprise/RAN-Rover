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
        void Spin(double speed, bool invert, bool filling);
        void Stop();
    private:
        double emptySpinSpeed = 0.4;
        double fillSpinSpeed = 0.1;
        bool isSpinning;
        bool isLocked;
        MotorSendable hopSpin{PortConstants::hopSpin, rev::CANSparkMax::MotorType::kBrushless};
};