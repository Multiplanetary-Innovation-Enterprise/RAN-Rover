#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <wpi/raw_ostream.h>
#include "Constants.h"

class HopperSubsystem : public frc2::SubsystemBase {
    public:
        double maxSpinSpeed = 0.15;
        
        HopperSubsystem();
        void Periodic() override;
        void SimulationPeriodic() override;
        void Reset();

        void Spin(double speed, bool invert);
        void Stop();
    private:
        bool isSpinning;
        rev::CANSparkMax hopSpin{PortConstants::hopSpin, rev::CANSparkMax::MotorType::kBrushless};
};