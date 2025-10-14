/**
 * @file ExcavationSubsystem.h
 * @brief Declaration of the ExcavationSubsystem class for rover excavation control.
 *
 * The ExcavationSubsystem class manages all hardware and logic related to the rover’s
 * excavation mechanism. This includes control of the spin motor, linear actuators for
 * raising and lowering the excavation arms, and the locking mechanism that stabilizes
 * the system during operation.
 *
 * The subsystem uses potentiometers for actuation feedback, digital outputs for actuator
 * control, and a CANSparkMax motor for spinning. It provides safety checks that prevent
 * motion outside of the defined actuation range and ensures spin only occurs when the
 * arms are in safe positions.
 *
 * This file is part of the Lunabotics rover control software, developed to support
 * autonomous excavation and regolith handling.
 *
 * @date 2025-10-14
 * @version 1.0
 * Last Updated By: Royce Doll
 *   Lunabotics Software Team — Michigan Technological University
 *
 * @see ExcavationSubsystem.cpp
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
 * @class ExcavationSubsystem
 * @brief Controls the excavation system of the rover, including spin, actuation, and locking mechanisms.
 *
 * This subsystem manages the digging arm or bucket mechanism used for excavation operations.
 * It provides methods to control spinning (for digging), actuating (raising/lowering), and
 * mechanical locking for stability. Position feedback is obtained using potentiometers.
 *
 * @note This subsystem uses digital outputs for actuation control and a CANSparkMax motor for spinning.
 */
class ExcavationSubsystem : public frc2::SubsystemBase {
    public:
    /**
     * @brief Constructs the ExcavationSubsystem and initializes motors and sensors.
     */
    ExcavationSubsystem();

    /**
     * @brief Periodic update function called every robot cycle.
     *
     * This can be used to publish telemetry, monitor sensor values, or update state variables.
     */
    void Periodic() override;

    /**
     * @brief Called periodically during simulation.
     *
     * Used to simulate actuator and sensor behavior when running in a simulated environment.
     */
    void SimulationPeriodic() override;

    /**
     * @brief Resets the excavation subsystem to a known state.
     *
     * Resets flags such as spinning, locking, and actuation direction.
     */
    void Reset();

    /**
     * @brief Determines whether the excavation system is in a safe position to spin.
     *
     * @return true if the system can safely spin, false otherwise.
     */
    bool canSpin();

    /**
     * @brief Controls the mechanical lock for the excavator arm.
     *
     * @param hold Set to true to engage the lock, false to release it.
     */
    void HoldLock(bool hold);

    /**
     * @brief Spins the excavation drum or digging mechanism.
     *
     * @param speed Speed value between 0.0 and 1.0 (subject to maxSpinSpeed).
     * @param invert If true, reverses the spin direction.
     */
    void Spin(double speed, bool invert);

    /**
     * @brief Stops the excavation spin motor.
     */
    void Stop();

    /**
     * @brief Begins actuation of the excavation arms.
     *
     * @param dir Direction of actuation.
     *        - true = extend (raise)
     *        - false = retract (lower)
     */
    void StartActuate(bool dir);

    /**
     * @brief Stops actuation of the excavation arms.
     *
     * @param limitHit Indicates whether a limit condition (min/max potentiometer value) was reached.
     */
    void StopActuate(bool limitHit);

    /**
     * @brief Retrieves the current average actuation angle from potentiometers.
     *
     * @return The average normalized actuation angle (0.0 to 1.0 range).
     */
    double ActuateAngle();
    private:
        double maxSpinSpeed = 0.25;         ///< Maximum speed at which the excavator can spin.
        double actuateSpeed = 0.025;        ///< Speed used for actuator movement.
        double actuateSpinThreshold = 0.2;  ///< Potentiometer threshold at which spinning is allowed during actuation.
        double actuateMin = 0.05;           ///< Minimum potentiometer limit for actuation.
        double actuateMax = 0.38;           ///< Maximum potentiometer limit for actuation.
        
        bool isSpinning;
        int actuatingDir;
        bool isLocked;
        MotorSendable excSpin{PortConstants::excSpin, rev::CANSparkMax::MotorType::kBrushless};

        frc::DigitalOutput excLeftActVel{PortConstants::excLeftActVel};
        frc::DigitalOutput excLeftActDir{PortConstants::excLeftActDir};
        frc::AnalogPotentiometer excLeftPot{PortConstants::excLeftPot, 1.0, 0.0};

        frc::DigitalOutput excRightActVel{PortConstants::excRightActVel};
        frc::DigitalOutput excRightActDir{PortConstants::excRightActDir};
        frc::AnalogPotentiometer excRightPot{PortConstants::excRightPot, 1.0, 0.0};
};
