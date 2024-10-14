#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <wpi/raw_ostream.h>
#include "Constants.h"
#include <array>

class MobilitySubsystem : public frc2::SubsystemBase {
    public:
        double maxDriveSpeed = 0.6;
        double maxCrawlSpeed = 0.1;
        double currentThreshold = 8.0;
        int cyclesSlipOff = 75; // 1.5 seconds (1500 ms @ 20ms cycles)
        int cyclesSlipRun = 75; // 1.5 seconds (1500 ms @ 20ms cycles)

        MobilitySubsystem();
        void Periodic() override;
        void SimulationPeriodic() override;
        void Reset();

        void Drive(double leftSpeed, double rightSpeed, bool crawl);
        void Stop();
    private:
        std::array<double, 4> SlipControl(std::array<double, 4> in);
        std::array<int, 4> slipWait;
        bool isSpinning, leftCrawl, rightCrawl;   

        rev::CANSparkMax backLeft{PortConstants::mobBackLeft, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax frontLeft{PortConstants::mobFrontLeft, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax backRight{PortConstants::mobBackRight, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax frontRight{PortConstants::mobFrontRight, rev::CANSparkMax::MotorType::kBrushless};
};