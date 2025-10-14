/**
 * @file MotorSendable.h
 * @brief Defines the MotorSendable class for exposing motor controller data to WPILib dashboards.
 *
 * This class extends the REV Robotics CANSparkMax motor controller and implements
 * the WPILib Sendable interface. It allows key motor data such as velocity, position,
 * and applied output to be published to SmartDashboard or Shuffleboard for live
 * monitoring and debugging.
 *
 * @note This class simplifies telemetry access for both drive and mechanism motors.
 * It assumes a single integrated encoder per motor (via SparkRelativeEncoder).
 *
 * @date October 2025
 * @version 1.0
 * Last Edited By: Royce Doll
 */
#pragma once

#include <rev/CANSparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableRegistry.h>

/**
 * @class MotorSendable
 * @brief Wrapper around the REV CANSparkMax motor controller that adds WPILib Sendable support.
 *
 * The MotorSendable class inherits from `rev::CANSparkMax` and exposes internal telemetry
 * such as encoder position, velocity, and applied output to WPILib dashboards. This is
 * especially useful for debugging drivetrain, steering, and actuator systems.
 */
class MotorSendable : public rev::CANSparkMax, public wpi::Sendable {
    public:
    /**
     * @brief Constructs a new MotorSendable object.
     *
     * @param deviceId The CAN device ID of the motor controller.
     * @param motorType The motor type (e.g., `rev::CANSparkMax::MotorType::kBrushless`).
     *
     * Automatically registers the motor as a Sendable object for dashboard display.
     */
    MotorSendable(int deviceId, MotorType motorType);

    /**
     * @brief Initializes the Sendable interface for WPILib dashboards.
     *
     * @param builder Reference to the SendableBuilder used to register dashboard properties.
     *
     * This method defines which motor properties (velocity, position, output, etc.)
     * will be displayed and updated in real time on SmartDashboard or Shuffleboard.
     */
    void InitSendable(wpi::SendableBuilder& builder) override;

    double gearRatio = 1;  ///< Gear ratio applied to the motor encoder measurements.
    rev::SparkRelativeEncoder encoder = GetEncoder();  ///< Integrated encoder for tracking motor position and velocity.
};
