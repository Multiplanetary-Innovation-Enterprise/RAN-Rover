
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"

class MAD : public frc2::SubsystemBase {
    public:
    CAM();

    void Periodic() override;
    int autoChassis(int CommandVar);
    void setVelocity(double leftvelocity, double rightvelocity);
    double getMADPosition();

    private:
    rev::CANSparkMax    m_rearRightDrive;
    rev::CANSparkMax    m_frontRightDrive;
    rev::CANSparkMax    m_rearLeftDrive;
    rev::CANSparkMax    m_frontLeftDrive;

    rev::SparkMaxRelativeEncoder m_rearRightEncoder;
    rev::SparkMaxRelativeEncoder m_frontRightEncoder;
    rev::SparkMaxRelativeEncoder m_rearLeftEncoder;
    rev::SparkMaxRelativeEncoder m_frontLeftEncoder;

}