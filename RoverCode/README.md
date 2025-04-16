# Rover Code 2024 - 2025

This is the rover code, written in C++ by Braeden Moore for the Astro Huskies's Lunabotics Rover "Atlas"

## How is this project organized?

Source files are in `src/main/cpp` and Header files are in `src/main/include`

Code entry begins in `Robot.cpp`  which initializes a singleton for each of the other subsystems which are in `src/main/cpp/subsystems` these subsystems include:
- ### Mobility
    - **Source:** `src/main/cpp/subsystems/Mobility.cpp`
    - **Header:** `src/main/include/subsystems/Mobility.h`
    - Start and Stop Functions for Wheel Motors.
    - Slip Control System that minimizes slippage when current drops in a motor.
    - TODO: Steer Function to move Individual Wheel Steering Actuators to specified angles.
- ### Hopper
    - **Source:** `src/main/cpp/subsystems/Hopper.cpp`
    - **Header:** `src/main/include/subsystems/Hopper.h`
    - Start and Stop Functions for Hopper Motor.
    - Hold Function to Lock Speed for Hopper Motor.
- ### Excavation
    - **Source:** `src/main/cpp/subsystems/Excavation.cpp`
    - **Header:** `src/main/include/subsystems/Excavation.h`
    - Start and Stop Functions for Excavation Motor.
    - Stop and Stop Actuation for Excavation Linear Actuators.
    - Constraints for Excavation Motor dependent on Linear Actuator Position.