#pragma once

#include "sendables/MotorSendable.h"

MotorSendable::MotorSendable(int deviceId, MotorType motorType) : MotorSendable::CANSparkMax(deviceId, motorType) {
    wpi::SendableRegistry::AddLW(this, "SparkMax", deviceId);
}

void MotorSendable::InitSendable(wpi::SendableBuilder& builder) {
    builder.SetActuator(true);
    // Gear Reduction is intended to be used as a divisor to the encoder RPM.
    builder.AddDoubleProperty("Gearbox Reduction", [this] { return gearRatio; }, [this] (double d) { if (d != 0) gearRatio = d; });
    builder.AddDoubleProperty("Encoder [RPM]", [this] { return encoder.GetVelocity() / gearRatio; }, [this] (double d) {} );
    builder.AddDoubleProperty("Current [Amps]", [this] { return this->GetOutputCurrent(); }, [this] (double d) {} );
}