#pragma once

#include <rev/CANSparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableRegistry.h>

class MotorSendable : public rev::CANSparkMax, public wpi::Sendable {
    public:
    double gearRatio = 1;
    rev::SparkRelativeEncoder encoder = GetEncoder();
    MotorSendable(int deviceId, MotorType motorType);
    void InitSendable(wpi::SendableBuilder& builder) override;
};