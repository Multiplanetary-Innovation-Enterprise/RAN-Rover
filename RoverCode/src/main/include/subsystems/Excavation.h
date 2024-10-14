#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <wpi/raw_ostream.h>
#include "Constants.h"

class ExcavationSubsystem : public frc2::SubsystemBase {
    public:
        double maxSpinSpeed = 0.25;

        ExcavationSubsystem();
        void Periodic() override;
        void SimulationPeriodic() override;
        void Reset();

        void Spin(double speed, bool invert);
        void Stop();
    private:
        bool isSpinning;
        rev::CANSparkMax excSpin{PortConstants::excSpin, rev::CANSparkMax::MotorType::kBrushless};
};