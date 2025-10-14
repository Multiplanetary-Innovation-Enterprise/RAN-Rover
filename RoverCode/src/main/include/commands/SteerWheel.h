/**
 * @file SteerWheel.h
 * @brief Defines the SteerWheel command used for steering control of the rover.
 *
 * This command interfaces with the MobilitySubsystem to control the steering angles
 * of the rover’s wheels. It supports multiple steering modes such as Ackermann steering,
 * crab (omnidirectional) steering, zero-point turns, and straight alignment.
 *
 * Each steering mode adjusts the target wheel angles accordingly, using encoder or
 * potentiometer feedback through the MobilitySubsystem.
 *
 * @note This class assumes the MobilitySubsystem manages motor actuation and sensor feedback.
 *
 * @date October 2025
 * @version 1.0
 * Last Edited By: Royce Doll
 */
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/DigitalOutput.h>
#include <frc/AnalogPotentiometer.h>
#include "subsystems/Mobility.h"

/**
 * @class SteerWheel
 * @brief Command for controlling wheel steering angles in various configurations.
 *
 * The SteerWheel command sets the desired steering angle for each wheel and supports
 * different steering modes such as Ackermann (turning), crab (sideways), zero-point rotation,
 * and straight alignment. It is designed for multi-wheel rovers that use independent
 * steering motors on each wheel.
 */
class SteerWheel : public frc2::CommandHelper<frc2::Command, SteerWheel> {
    public:
        /**
         * @brief Constructs a new SteerWheel command.
         *
         * @param mob Pointer to the MobilitySubsystem responsible for steering control.
         */
        explicit SteerWheel(MobilitySubsystem* mob);
         /**
          * @brief Called once when the command is initially scheduled.
          *
          * Initializes steering targets or resets wheel orientation tracking as needed.
          */
        void Initialize() override;
         /**
          * @brief Called repeatedly while the command is active.
          *
          * Continuously adjusts wheel angles toward the target values based on the current steering mode.
          */
        void Execute() override;
        /**
         * @brief Checks if the steering command has finished execution.
         *
         * @return true if all wheels have reached their target angles.
         * @return false otherwise.
         */
        bool IsFinished() override;
        /**
         * @brief Called once after the command ends or is interrupted.
         *
         * @param interrupted True if the command was interrupted by another command.
         *
         * Typically stops steering motor movement and resets control outputs.
         */
        void End(bool interrupted) override;
        /**
         * @brief Configures wheel angles for Ackermann steering.
         *
         * @param angle Desired steering angle (e.g., in degrees or radians).
         *
         * Each wheel’s angle is calculated according to Ackermann geometry for smooth turns.
         */
        void Ackermann(double angle);
        /**
         * @brief Configures wheel angles for crab (omnidirectional) steering.
         *
         * @param angle Desired uniform steering angle for all wheels.
         *
         * All wheels are aligned to the same angle, allowing lateral (sideways) movement.
         */
        void Crab(double angle);
        /**
         * @brief Configures wheel angles for zero-point turning.
         *
         * Each wheel turns toward a calculated orientation that allows the rover
         * to rotate about its center without translational movement.
         */
        void ZeroPoint();
        /**
         * @brief Aligns all wheels straight forward.
         *
         * Useful for initializing or resetting steering to a known position.
         */
        void Straight();
    private:
        MobilitySubsystem* mob;                           ///< Pointer to the MobilitySubsystem for steering control.

        double angleTarget[4] = {0.0, 0.0, 0.0, 0.0};     ///< Target steering angles for each wheel.
        double angleMargin[4] = {1.0, 1.0, 1.0, 1.0};     ///< Acceptable error margins for each wheel angle.
        bool dir[4] = {false, false, false, false};       ///< Direction flags for each wheel’s steering motor.
};
