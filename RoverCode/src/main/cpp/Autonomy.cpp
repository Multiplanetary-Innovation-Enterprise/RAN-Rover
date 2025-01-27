#include "Autonomy.h"
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

Autonomy::Autonomy(Robot* robot, Vision* vision, MobilitySubsystem* mob, ExcavationSubsystem* exc, HopperSubsystem* hop)
    : robot(robot), vision(vision), mob(mob), exc(exc), hop(hop) {

    frc::SmartDashboard::PutBoolean("Auto Traversal", true);
    frc::SmartDashboard::SetPersistent("Auto Traversal");
    frc::SmartDashboard::PutBoolean("Auto EXC Orient", true);
    frc::SmartDashboard::SetPersistent("Auto EXC Orient");
    frc::SmartDashboard::PutBoolean("Auto EXC Action", true);
    frc::SmartDashboard::SetPersistent("Auto EXC Action");
    frc::SmartDashboard::PutBoolean("Auto DEP Orient", true);
    frc::SmartDashboard::SetPersistent("Auto DEP Orient");
    frc::SmartDashboard::PutBoolean("Auto DEP Action", true);
    frc::SmartDashboard::SetPersistent("Auto DEP Action");

    // Adjustable Autonomous Variables

    // Mobility
    frc::SmartDashboard::PutNumber("Zero Point Drive Speed", zeroPointDriveSpeed);
    frc::SmartDashboard::SetPersistent("Zero Point Drive Speed");

    // Exc Action
    frc::SmartDashboard::PutNumber("Excavation Spin Speed", excavationSpinSpeed);
    frc::SmartDashboard::SetPersistent("Excavation Spin Speed");
    frc::SmartDashboard::PutNumber("Excavation Spin Start Angle", excavationSpinStartAngle);
    frc::SmartDashboard::SetPersistent("Excavation Spin Start Angle");
    frc::SmartDashboard::PutNumber("Excavation Stow Angle", excavationStowAngle);
    frc::SmartDashboard::SetPersistent("Excavation Stow Angle");
    frc::SmartDashboard::PutNumber("Excavation Cycle Time", excavationCycleTime.value());
    frc::SmartDashboard::SetPersistent("Excavation Cycle Time");
    frc::SmartDashboard::PutNumber("Hopper Index Speed", hopperIndexSpeed);
    frc::SmartDashboard::SetPersistent("Hopper Index Speed");
    frc::SmartDashboard::PutNumber("Hopper Index Time", hopperIndexTime.value());
    frc::SmartDashboard::SetPersistent("Hopper Index Time");
    frc::SmartDashboard::PutNumber("Hopper Index Period", hopperIndexPeriod.value());
    frc::SmartDashboard::SetPersistent("Hopper Index Period");

    // Dep Action
    frc::SmartDashboard::PutNumber("Hopper Empty Speed", hopperEmptySpeed);
    frc::SmartDashboard::SetPersistent("Hopper Empty Speed");
    frc::SmartDashboard::PutNumber("Hopper Empty Time", hopperEmptyTime.value());
    frc::SmartDashboard::SetPersistent("Hopper Empty Time");
}

void Autonomy::Init() {

    currentPhase = TRAVERSAL;

    if (frc::SmartDashboard::GetBoolean("Auto Traversal", false))
        TraversalInit();
    if (frc::SmartDashboard::GetBoolean("Auto EXC Orient", false))
        ExcOrientInit();
    if (frc::SmartDashboard::GetBoolean("Auto EXC Action", false))
        ExcActionInit();
    if (frc::SmartDashboard::GetBoolean("Auto DEP Orient", false))
        DepOrientInit();
    if (frc::SmartDashboard::GetBoolean("Auto DEP Action", false))
        DepActionInit();
}

void Autonomy::Periodic() {

    vision->IdentifyTags();

    switch(currentPhase) {
        case TRAVERSAL:
            if (frc::SmartDashboard::GetBoolean("Auto Traversal", false) && TraversalPeriodic())
                currentPhase = EXC_ORIENT;
            break;
        case EXC_ORIENT:
            if (frc::SmartDashboard::GetBoolean("Auto EXC Orient", false) && ExcOrientPeriodic())
                currentPhase = EXC_ACTION;
            break;
        case EXC_ACTION:
            if (frc::SmartDashboard::GetBoolean("Auto EXC Action", false) && ExcActionPeriodic())
                currentPhase = DEP_ORIENT;
            break;
        case DEP_ORIENT:
            if (frc::SmartDashboard::GetBoolean("Auto DEP Orient", false) && DepOrientPeriodic())
                currentPhase = DEP_ACTION;
            break;
        case DEP_ACTION:
            if (frc::SmartDashboard::GetBoolean("Auto DEP Action", false) && DepActionPeriodic())
                robot->Kill();
            break;
    }
}

void Autonomy::TraversalInit() {
    if (!frc::SmartDashboard::GetBoolean("Auto Traversal", false)) {
        fmt::print("Skipping Auto Traversal Init\n");
        return;
    }
}
bool Autonomy::TraversalPeriodic() {
    if (!frc::SmartDashboard::GetBoolean("Auto Traversal", false)) {
        fmt::print("Skipping Auto Traversal Periodic\n");
        return true;
    }

    if (FindBeacon(1))
        NavigateToBeacon(1);
}

void Autonomy::ExcOrientInit() {
    if (!frc::SmartDashboard::GetBoolean("Auto EXC Orient", false)) {
        fmt::print("Skipping Auto EXC Orient Init\n");
        return;
    }
}
bool Autonomy::ExcOrientPeriodic() {
    if (!frc::SmartDashboard::GetBoolean("Auto EXC Orient", false)) {
        fmt::print("Skipping Auto EXC Orient Periodic\n");
        return true;
    }

    return true;
}

void Autonomy::ExcActionInit() {
    if (!frc::SmartDashboard::GetBoolean("Auto EXC Action", false)) {
        fmt::print("Skipping Auto EXC Action Init\n");
        return;
    }
    
    excavationSpinSpeed = frc::SmartDashboard::GetNumber("Excavation Spin Speed", excavationSpinSpeed);
    excavationSpinStartAngle = frc::SmartDashboard::GetNumber("Excavation Spin Start Angle", excavationSpinStartAngle);
    excavationStowAngle = frc::SmartDashboard::GetNumber("Excavation Stow Angle", excavationStowAngle);
    excavationCycleTime = units::time::second_t{frc::SmartDashboard::GetNumber("Excavation Cycle Time", excavationCycleTime.value())};

    hopperIndexSpeed = frc::SmartDashboard::GetNumber("Hopper Index Speed", hopperIndexSpeed);
    hopperIndexTime = units::time::second_t{frc::SmartDashboard::GetNumber("Hopper Index Time", hopperIndexTime.value())};
    hopperIndexPeriod = units::time::second_t{frc::SmartDashboard::GetNumber("Hopper Index Period", hopperIndexPeriod.value())};
}
bool Autonomy::ExcActionPeriodic() {
    if (!frc::SmartDashboard::GetBoolean("Auto EXC Action", false)) {
        fmt::print("Skipping Auto EXC Action Periodic\n");
        return true;
    }

    timerA.Start();
    if (!timerA.HasElapsed(excavationCycleTime)) {
        if (exc->ActuateAngle() < excavationSpinStartAngle)
            exc->StartActuate(true);
        else {
            exc->StopActuate(true);
            exc->Spin(excavationSpinSpeed, false);

            // Hopper Indexing
            timerB.Start();
            if (timerB.HasElapsed(hopperIndexPeriod)) {
                hop->Spin(hopperIndexSpeed, false);
                if (timerB.HasElapsed(hopperIndexPeriod + hopperIndexTime)) {
                    hop->Stop();
                    timerB.Stop();
                }
            }
        }
    } else {
        timerA.Stop();
        if (exc->ActuateAngle() > excavationStowAngle) {
            exc->StartActuate(true);
        } else {
            exc->StopActuate(true);
            return true;
        }
    }

    return false;
}

void Autonomy::DepOrientInit() {
    if (!frc::SmartDashboard::GetBoolean("Auto DEP Orient", false)) {
        fmt::print("Skipping Auto DEP Orient Init\n");
        return;
    }
}
bool Autonomy::DepOrientPeriodic() {
    if (!frc::SmartDashboard::GetBoolean("Auto DEP Orient", false)) {
        fmt::print("Skipping Auto DEP Orient Periodic\n");
        return true;
    }

    return true;
}

void Autonomy::DepActionInit() {
    if (!frc::SmartDashboard::GetBoolean("Auto DEP Action", false)) {
        fmt::print("Skipping Auto DEP Action Init\n");
        return;
    }

    hopperEmptySpeed = frc::SmartDashboard::GetNumber("Hopper Empty Speed", hopperEmptySpeed);
    hopperEmptyTime = units::time::second_t{frc::SmartDashboard::GetNumber("Hopper Empty Time", hopperEmptyTime.value())};
}
bool Autonomy::DepActionPeriodic() {
    if (!frc::SmartDashboard::GetBoolean("Auto DEP Action", false)) {
        fmt::print("Skipping Auto DEP Action Periodic\n");
        return true;
    }
    // TODO: Will need to be changed to dep when we switch to Atlas
    // Spin Deposition Hopper so the belt has done one full rotation.
    timerA.Start();
    hop->Spin(hopperEmptySpeed, false);
    if (timerA.HasElapsed(hopperEmptyTime)) {
        timerA.Stop();
        hop->Stop();
        return true;
    }
    return false;
}

/////////////////////////////////
// Autonomous Helper Functions //
/////////////////////////////////

bool Autonomy::FindBeacon(int tagId) {

    if (vision->isTagVisible(tagId)) {
        // Found Beacon
        return true;
    }

    // Assume that tags are ordered clockwise in numerical order around the rover.
    std::vector<int> visibleTags = vision->getVisibleTags();
    if (visibleTags.size() <= 1) { // One or zero tags visible, use that to inform direction of where target tag is.
         int tagLastSeen = vision->getTagLastSeen();
        if (tagLastSeen > tagId) { // Last tag seen is to the right of where our target should be, turn counter-clockwise.
            // Zero-Point Turn Counter-Clockwise
        } else { // Last tag seen is to the left of where our target should be, or we haven't seen a tag yet, turn clockwise.
            // Zero-Point Turn Clockwise
        }
    } else { // Two tags visible without finding target beacon, we need to reposition because our target must be visually obstructed.
        // Reposition
    }
}

bool Autonomy::NavigateToBeacon(int tagId) {

    if (!vision->isTagVisible(tagId)) {
        // Locate Beacon
        return false;
    }

    double angle = vision->getTagAngle(tagId);
    double normal = vision->getTagNormal(tagId);
    double distance = vision->getTagDistance(tagId);

    if (abs(normal) < 25) { // Normal allows for direct path
        if (abs(normal) < 5) { // Rover is aligned with normal of beacon
            wpi::outs() << "1: Crab to beacon\n";

            // Steer wheels to angle
            steer.Crab(angle);
            if (!steer.IsFinished()) {
            
                wpi::outs() << "Steering Wheels to Angle " << std::to_string(angle) << "°, currently at:\n" <<
                                std::to_string(int(mob->GetAngle(MobilitySubsystem::Wheel::FRONT_LEFT))) << "°  " << std::to_string(int(mob->GetAngle(MobilitySubsystem::Wheel::FRONT_RIGHT))) << "°\n" <<
                                std::to_string(int(mob->GetAngle(MobilitySubsystem::Wheel::BACK_LEFT))) << "°  " << std::to_string(int(mob->GetAngle(MobilitySubsystem::Wheel::BACK_RIGHT))) << "°\n";
                if (!steer.IsScheduled()) {
                    robot->Kill();
                    steer.Schedule();
                }
            
            // If wheel angle is correct
            } else {    

                // Drive so distance is small
                distDrive.Set(distance);
                if (!distDrive.IsFinished()) {

                    wpi::outs() << "Distance Drive currently at " << std::to_string(distance) << "\n";
                    if (!distDrive.IsScheduled()) {
                        robot->Kill();
                        distDrive.Schedule();
                    }

                // If distance is small
                } else {

                    wpi::outs() << "Distance Drive Done, Should be in position\n";     
                    robot->Kill();    
                    return true;
                }
            }
        } else {
            wpi::outs() << "2: Turn to align angle and normal\n";
            // Steer wheels to zero-point configuration
            steer.ZeroPoint();
            if (!steer.IsFinished()) {

                wpi::outs() << "Steering Wheels to Zero-Point Configuration, currently at:\n" <<
                                std::to_string(int(mob->GetAngle(MobilitySubsystem::Wheel::FRONT_LEFT))) << "°  " << std::to_string(int(mob->GetAngle(MobilitySubsystem::Wheel::FRONT_RIGHT))) << "°\n" <<
                                std::to_string(int(mob->GetAngle(MobilitySubsystem::Wheel::BACK_LEFT))) << "°  " << std::to_string(int(mob->GetAngle(MobilitySubsystem::Wheel::BACK_RIGHT))) << "°\n";
                if (!steer.IsScheduled()) {
                    robot->Kill();
                    steer.Schedule();
                }
            } else {
                // If wheel angle is correct
                //    Zero-point turn until angle = normal
                if (normal > 0)
                    mob->Drive({0.3, -0.3, 0.3, -0.3});
                else
                    mob->Drive({-0.3, 0.3, -0.3, 0.3});
            }
        }
    } else { // Will need to reposition to have a direct path to beacon
        wpi::outs() << "3: Facing towards beacon but too close, need to move away so we can reposition\n";
        // Steer wheels to face away
        steer.Crab((angle / abs(angle)) * 45.0);
        if (!steer.IsFinished()) {
            if (!steer.IsScheduled()) {
                robot->Kill();
                steer.Schedule();
            }
        } else {
            // If wheel angle is correct
            mob->Drive({0.4, 0.4, 0.4, 0.4});
        }
    }
}

bool Autonomy::ZeroPointTurn(double degrees) {

    // Turn Wheels to be in ZeroPoint Configuration
    steer.ZeroPoint();
    if (!steer.IsFinished()) {
        if (!steer.IsScheduled()) {
            robot->Kill();
            steer.Schedule();
        }
        return false;
    }
    // Retrieves the current yaw, will look for this to be equal to 'degrees'. 
    float yaw = imu.GetYaw(); 
    if (abs(degrees - yaw) > 5.0) { // Margin of +/- 5 degrees to target.

        zeroPointDriveSpeed = frc::SmartDashboard::GetNumber("Zero Point Drive Speed", zeroPointDriveSpeed);

        if (int(degrees - yaw) % 360 >= 180) { // Needs to turn right
            mob->Drive({zeroPointDriveSpeed, -zeroPointDriveSpeed, zeroPointDriveSpeed, -zeroPointDriveSpeed});
        } else if (int(degrees - yaw) % 360 < 180) { // Neds to turn left
            mob->Drive({-zeroPointDriveSpeed, zeroPointDriveSpeed, -zeroPointDriveSpeed, zeroPointDriveSpeed});
        }

        return false;
    }

    steer.Straight();
    if (!steer.IsFinished()) {
        if (!steer.IsScheduled()) {
            robot->Kill();
            steer.Schedule();
        }
        return false;
    }

    return true;
} 