#pragma once

#include "sendables/IMUSendable.h"

IMUSendable::IMUSendable(frc::SPI::Port port) : IMUSendable::AHRS(port) {
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
    // Tests have shown Temperature is not entirely accurate, may need another method of collecting temperature data.
    builder.AddDoubleProperty("Temperature [C]", [this] { return GetTempC(); }, [this] (double d) {} );
}