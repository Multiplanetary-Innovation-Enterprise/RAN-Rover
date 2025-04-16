#include "sendables/IMUSendable.h"

IMUSendable::IMUSendable(frc::SerialPort::Port port) : IMUSendable::AHRS(port, AHRS::SerialDataType::kProcessedData, 50.0) {
    wpi::SendableRegistry::Add(this, "NavX AHRS", port);
}

void IMUSendable::InitSendable(wpi::SendableBuilder& builder) {
    builder.SetActuator(true);
    builder.AddDoubleProperty("Acceleration X [G]", [this] { return GetWorldLinearAccelX(); }, [this] (double d) {} );
    builder.AddDoubleProperty("Acceleration Y [G]", [this] { return GetWorldLinearAccelY(); }, [this] (double d) {} );
    builder.AddDoubleProperty("Acceleration Z [G]", [this] { return GetWorldLinearAccelZ(); }, [this] (double d) {} );
    builder.AddDoubleProperty("Velocity X [mps]", [this] { return GetVelocityX(); }, [this] (double d) {} );
    builder.AddDoubleProperty("Velocity Y [mps]", [this] { return GetVelocityY(); }, [this] (double d) {} );
    builder.AddDoubleProperty("Velocity Z [mps]", [this] { return GetVelocityZ(); }, [this] (double d) {} );
}