#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <wpi/raw_ostream.h>
#include "Constants.h"
#include <array>
#include "sendables/MotorSendable.h"

class MobilitySubsystem : public frc2::SubsystemBase {
    public:
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

        double maxDriveSpeed = 0.6;
        double maxCrawlSpeed = 0.1;
        double currentThreshold = 8.0;
        int cyclesSlipOff = 75; // 1.5 seconds (1500 ms @ 20ms cycles)
        int cyclesSlipTry = 75; // 1.5 seconds (1500 ms @ 20ms cycles)

        MotorSendable backLeft{PortConstants::mobBackLeft, rev::CANSparkMax::MotorType::kBrushless};
        MotorSendable frontLeft{PortConstants::mobFrontLeft, rev::CANSparkMax::MotorType::kBrushless};
        MotorSendable backRight{PortConstants::mobBackRight, rev::CANSparkMax::MotorType::kBrushless};
        MotorSendable frontRight{PortConstants::mobFrontRight, rev::CANSparkMax::MotorType::kBrushless};
};