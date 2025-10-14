/**
 * @file IMUSendable.h
 * @brief Defines the IMUSendable class for exposing AHRS (IMU) data to WPILib dashboards.
 *
 * This class extends the AHRS gyro/IMU interface and implements the WPILib Sendable
 * interface, allowing inertial data (yaw, pitch, roll, etc.) to be published to
 * SmartDashboard or Shuffleboard for live monitoring and debugging.
 *
 * @note Typically used with a navX-MXP or similar AHRS connected via SPI.
 *
 * @date October 2025
 * @version 1.0
 * Last Updated By: Royce Doll
 */
#pragma once

#include <AHRS.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableRegistry.h>

/**
 * @class IMUSendable
 * @brief Wrapper around the AHRS sensor that adds Sendable support for dashboard integration.
 *
 * This class allows the IMU’s readings (such as yaw, pitch, and roll) to be displayed
 * on SmartDashboard or Shuffleboard in real time. It inherits from the AHRS class and
 * overrides the InitSendable() method to define how sensor data is exposed.
 */
class IMUSendable : public AHRS {
    public:
    /**
     * @brief Constructs a new IMUSendable object.
     *
     * @param port The SPI port used to connect the AHRS (typically `frc::SPI::Port::kMXP`).
     */
    IMUSendable(frc::SPI::Port port);
    /**
     * @brief Initializes the Sendable interface for WPILib dashboards.
     *
     * @param builder Reference to the SendableBuilder used to register dashboard properties.
     *
     * This function is automatically called by WPILib when the Sendable is added
     * to a dashboard. It defines which IMU properties (yaw, pitch, roll, etc.)
     * are published for live updates.
     */
    void InitSendable(wpi::SendableBuilder& builder) override;
};
