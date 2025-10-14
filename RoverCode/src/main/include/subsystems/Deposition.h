/**
 * @file Deposition.h
 * @brief Defines the DepositionSubsystem for controlling the rover’s material deposition mechanism.
 *
 * The DepositionSubsystem manages the regolith deposition system, including the spinning
 * drum and actuation mechanism used to tilt or release collected material. It handles
 * motion limits via potentiometers, direction control through digital outputs, and
 * speed control through a CANSparkMax motor.
 *
 * Safety constraints (such as spin enable thresholds and actuation limits) are built in
 * to prevent mechanical interference during operation.
 *
 * @note This subsystem should not operate simultaneously with excavation mechanisms
 * beyond defined actuation thresholds.
 *
 * @date October 2025
 * @version 1.0
 * Last Updated By: Royce Doll
 */
#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <wpi/raw_ostream.h>
#include <frc/DigitalOutput.h>
#include <frc/AnalogPotentiometer.h>
#include "Constants.h"
#include "sendables/MotorSendable.h"

/**
 * @class DepositionSubsystem
 * @brief Controls the regolith deposition mechanism, including spin and actuation.
 *
 * This subsystem manages both the spinning mechanism used for material release and
 * the actuators that control the tilt or gate of the deposition system. Potentiometers
 * are used to monitor actuator position and enforce safe movement limits.
 */
class DepositionSubsystem : public frc2::SubsystemBase {
    public:
    DepositionSubsystem();
    /**
     * @brief Constructs a new DepositionSubsystem.
     *
     * Initializes motors, potentiometers, and digital outputs for controlling
     * the deposition system hardware.
     */
    DepositionSubsystem();

    /**
     * @brief Called periodically during robot operation.
     *
     * Used to update sensor readings and perform continuous control or telemetry updates.
     */
    void Periodic() override;

    /**
     * @brief Called periodically during simulation mode.
     *
     * Used to simulate actuator positions and motor behavior for testing in non-hardware environments.
     */
    void SimulationPeriodic() override;

    /**
     * @brief Resets the subsystem state.
     *
     * Stops all motion, resets flags, and prepares the subsystem for a new operation.
     */
    void Reset();

    /**
     * @brief Checks if spinning is currently allowed.
     *
     * @return true if the actuator position is within the safe spin range.
     * @return false if spinning would cause interference.
     */
    bool canSpin();

    /**
     * @brief Spins the deposition drum at the specified speed.
     *
     * @param speed Desired spin speed (normalized -1.0 to 1.0).
     * @param invert If true, reverses the spin direction.
     *
     * The spin speed is limited by `maxSpinSpeed` for safety.
     */
    void Spin(double speed, bool invert);

    /**
     * @brief Immediately stops the spin motor.
     */
    void Stop();

    /**
     * @brief Starts actuator motion in the specified direction.
     *
     * @param dir Direction flag: true for forward, false for reverse.
     *
     * Activates actuator motor control lines to begin tilting the deposition mechanism.
     */
    void StartActuate(bool dir);

    /**
     * @brief Stops actuator motion.
     *
     * @param limitHit True if motion was stopped due to reaching a limit switch or potentiometer boundary.
     *
     * Ensures that actuator motion ceases safely and prevents overtravel.
     */
    void StopActuate(bool limitHit);


    private:
        double maxSpinSpeed = 0.25;         ///< Maximum allowed spin speed for the drum motor.
        double actuateSpeed = 0.025;        ///< Base speed for actuator motion.
        double actuateSpinThreshold = 0.15; ///< Minimum actuator position required to allow spinning.
        double actuateMin = 0.1;            ///< Lower limit for actuator potentiometer readings.
        double actuateMax = 0.35;           ///< Upper limit for actuator potentiometer readings.

        // --- State variables ---
        bool isSpinning;     ///< Tracks whether the drum is currently spinning.
        int actuatingDir;    ///< Current direction of actuator movement

        // --- Hardware components ---
        MotorSendable depSpin{PortConstants::depSpin, rev::CANSparkMax::MotorType::kBrushless}; ///< Motor controlling the deposition drum.

        // Left actuator components
        frc::DigitalOutput depLeftActVel{PortConstants::depLeftActVel};           ///< Left actuator velocity control pin.
        frc::DigitalOutput depLeftActDir{PortConstants::depLeftActDir};           ///< Left actuator direction control pin.
        frc::AnalogPotentiometer depLeftPot{PortConstants::depLeftPot, 1.0, 0.0}; ///< Left actuator position sensor.

        // Right actuator components
        frc::DigitalOutput depRightActVel{PortConstants::depRightActVel};           ///< Right actuator velocity control pin.
        frc::DigitalOutput depRightActDir{PortConstants::depRightActDir};           ///< Right actuator direction control pin.
        frc::AnalogPotentiometer depRightPot{PortConstants::depRightPot, 1.0, 0.0}; ///< Right actuator position sensor.
};
