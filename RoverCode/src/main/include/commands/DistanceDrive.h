/**
 * @file DistanceDrive.h
 * @brief Defines the DistanceDrive command for driving the robot a specific distance.
 *
 * This command interfaces with the MobilitySubsystem to move the rover forward or backward
 * a given distance using encoder feedback. It supports smooth deceleration near the target
 * and can be configured dynamically via the Set() method.
 *
 * @note This command assumes that the MobilitySubsystem handles encoder-based distance measurement
 * and drive motor control.
 *
 * @date October 2025
 * @version 1.0
 * Last Edited By: Royce Doll
 */

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Mobility.h"

/**
 * @class DistanceDrive
 * @brief Command for driving the robot a given distance using encoder feedback.
 *
 * The DistanceDrive command calculates the desired travel distance and drives
 * the robot in a straight line until the target distance is reached. It automatically
 * slows down as it approaches the destination for smoother stopping.
 */

class DistanceDrive : public frc2::CommandHelper<frc2::Command, DistanceDrive> {
    public:
        /**
         * @brief Constructs a new DistanceDrive command.
         * 
         * @param mob Pointer to the MobilitySubsystem used for driving.
         */
        explicit DistanceDrive(MobilitySubsystem* mob);
        /**
         * @brief Called once when the command is initially scheduled.
         *
         * Resets encoders or initializes distance tracking as needed.
         */
        void Initialize() override;
        /**
         * @brief Called repeatedly while the command is active.
         *
         * Continuously drives the robot toward the target distance. May include
         * logic to decelerate as the robot nears its goal.
         */
        void Execute() override;
        /**
         * @brief Checks whether the command has finished execution.
         *
         * @return true if the robot has reached the target distance.
         * @return false otherwise.
         */
        bool IsFinished() override;
         /**
         * @brief Called once after the command ends or is interrupted.
         * 
         * @param interrupted True if the command was preempted by another command.
         *
         * Stops the robot and performs any cleanup operations.
         */
        void End(bool interrupted) override;
        /**
         * @brief Sets the target travel distance for the command.
         * 
         * @param dist Desired distance to drive (in appropriate units, e.g. inches or meters).
         */
        void Set(double dist);
    private:
        MobilitySubsystem* mob;    ///< Pointer to the MobilitySubsystem for drive control.

        double dist = 0.0;         ///< Target distance to drive.
        double distSlow = 40.0;    ///< Distance threshold for slowing down.
        double distLimit = 20.0;   ///< Minimum distance at which fine control may engage.
};
