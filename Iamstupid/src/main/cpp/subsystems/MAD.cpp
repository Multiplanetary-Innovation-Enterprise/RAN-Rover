

#include "subsystems/MAD.h"
#include <iostream>
#include <string.h>
#include "Constants.h"

CAM::CAM() :

m_rearRightDrive{MADConstants::rearRightDriveID, rev::CANSparkMax::MotorType::kBrushless},
m_frontRightDrive{MADConstants::frontRightDriveID, rev::CANSparkMax::MotorType::kBrushless},
m_rearLeftDrive{MADConstants::rearLeftDriveID, rev::CANSparkMax::MotorType::kBrushless},
m_frontLefttDrive{MADConstants::frontLeftDriveID, rev::CANSparkMax::MotorType::kBrushless},

m_rearRightEncoder{m_rearRightDrive.GetEncoder()},
m_frontRightEncoder{m_frontRightDrive.GetEncoder()},
m_rearLeftEncoder{m_rearLeftDrive.getEncoder()},
m_frontLeftEncoder{m_frontLeftDrive.getEncoder()} {}

void MAD::Periodic() {}

