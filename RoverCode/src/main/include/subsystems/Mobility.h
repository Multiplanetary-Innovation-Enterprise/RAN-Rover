#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <wpi/raw_ostream.h>
#include <array>
#include <frc/DigitalOutput.h>
#include <frc/AnalogPotentiometer.h>
#include "Constants.h"
#include "sendables/MotorSendable.h"


class MobilitySubsystem : public frc2::SubsystemBase {
    public:

        enum Wheel {
            FRONT_LEFT = 0,
            FRONT_RIGHT = 1,
            BACK_LEFT = 2,
            BACK_RIGHT = 3
        };

        MobilitySubsystem();
        void Periodic() override;
        void SimulationPeriodic() override;
        void Reset();

        void Drive(std::array<double, 4> speed);
        void Crawl(bool forward);
        void StopAll();

        void StartActuate(Wheel w, bool dir);
        void StopActuate(Wheel w, bool limitHit);

        // Provides angle in degrees of wheel steering
        double GetAngle(Wheel w);
    private:
        std::array<double, 4> SlipControl(std::array<double, 4> in);
        std::array<int, 4> slipWait;
        bool isSpinning, leftCrawl, rightCrawl;   

        double maxDriveSpeed = 0.6;
        double maxCrawlSpeed = 0.1;
        double currentThreshold = 8.0;
        int cyclesSlipOff = 75; // 1.5 seconds (1500 ms @ 20ms cycles)
        int cyclesSlipTry = 75; // 1.5 seconds (1500 ms @ 20ms cycles)
        int actuateDirs[4] = {0, 0, 0, 0};

        // Actuate Limits {Min, Max}
        double actuateLimits[4][2] = {{0.0, 1.0},
                                      {0.0, 1.0},
                                      {0.0, 1.0},
                                      {0.0, 1.0}};

        MotorSendable motor[4] = {MotorSendable(PortConstants::mobMotor[0], rev::CANSparkMax::MotorType::kBrushless),
                                  MotorSendable(PortConstants::mobMotor[1], rev::CANSparkMax::MotorType::kBrushless),
                                  MotorSendable(PortConstants::mobMotor[2], rev::CANSparkMax::MotorType::kBrushless),
                                  MotorSendable(PortConstants::mobMotor[3], rev::CANSparkMax::MotorType::kBrushless)};
        frc::DigitalOutput actVel[4] = {frc::DigitalOutput(PortConstants::mobActVel[0]),
                                        frc::DigitalOutput(PortConstants::mobActVel[1]),
                                        frc::DigitalOutput(PortConstants::mobActVel[2]),
                                        frc::DigitalOutput(PortConstants::mobActVel[3])};
        frc::DigitalOutput actDir[4] = {frc::DigitalOutput(PortConstants::mobActDir[0]),
                                        frc::DigitalOutput(PortConstants::mobActDir[1]),
                                        frc::DigitalOutput(PortConstants::mobActDir[2]),
                                        frc::DigitalOutput(PortConstants::mobActDir[3])};
        frc::AnalogPotentiometer actPot[4] = {frc::AnalogPotentiometer(PortConstants::mobActPot[0], 1.0, 0.0),
                                              frc::AnalogPotentiometer(PortConstants::mobActPot[1], 1.0, 0.0),
                                              frc::AnalogPotentiometer(PortConstants::mobActPot[2], 1.0, 0.0),
                                              frc::AnalogPotentiometer(PortConstants::mobActPot[3], 1.0, 0.0)};
};