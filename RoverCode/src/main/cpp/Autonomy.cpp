#include "Autonomy.h"
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/TimedRobot.h>

Autonomy::Autonomy() {
    
    frc::SmartDashboard::PutBoolean("Autonomy/Phases/Traversal", true);
    frc::SmartDashboard::SetPersistent("Autonomy/Phases/Traversal");
    frc::SmartDashboard::PutBoolean("Autonomy/Phases/EXC Orient", true);
    frc::SmartDashboard::SetPersistent("Autonomy/Phases/EXC Orient");
    frc::SmartDashboard::PutBoolean("Autonomy/Phases/EXC Action", true);
    frc::SmartDashboard::SetPersistent("Autonomy/Phases/EXC Action");
    frc::SmartDashboard::PutBoolean("Autonomy/Phases/DEP Orient", true);
    frc::SmartDashboard::SetPersistent("Autonomy/Phases/DEP Orient");
    frc::SmartDashboard::PutBoolean("Autonomy/Phases/DEP Action", true);
    frc::SmartDashboard::SetPersistent("Autonomy/Phases/DEP Action");
    frc::SmartDashboard::PutBoolean("Autonomy/Phases/Loop", true);
    frc::SmartDashboard::SetPersistent("Autonomy/Phases/Loop");
    
    // Adjustable Autonomous Variables
    
    frc::SmartDashboard::PutNumber("Autonomy/Traversal/DriveTimePerCell", driveTimePerCell);
    frc::SmartDashboard::SetPersistent("Autonomy/Traversal/DriveTime Factor");
    frc::SmartDashboard::PutNumber("Autonomy/Traversal/TurnTime 45", turnTimes[0].value());
    frc::SmartDashboard::SetPersistent("Autonomy/Traversal/TurnTime 45");
    frc::SmartDashboard::PutNumber("Autonomy/Traversal/TurnTime 90", turnTimes[1].value());
    frc::SmartDashboard::SetPersistent("Autonomy/Traversal/TurnTime 90");
    frc::SmartDashboard::PutNumber("Autonomy/Traversal/TurnTime 135", turnTimes[2].value());
    frc::SmartDashboard::SetPersistent("Autonomy/Traversal/TurnTime 135");
    frc::SmartDashboard::PutNumber("Autonomy/Traversal/TurnTime 180", turnTimes[3].value());
    frc::SmartDashboard::SetPersistent("Autonomy/Traversal/TurnTime 180");
    frc::SmartDashboard::PutNumber("Autonomy/Traversal/Path Start Distance", pathStartDistance);
    frc::SmartDashboard::SetPersistent("Autonomy/Traversal/Path Start Distance");
    frc::SmartDashboard::PutNumber("Autonomy/Traversal/Path Distance", pathDistance);
    frc::SmartDashboard::SetPersistent("Autonomy/Traversal/Path Distance");
    frc::SmartDashboard::PutNumber("Autonomy/Traversal/To Berm Distance", toBermDistance);
    frc::SmartDashboard::SetPersistent("Autonomy/Traversal/To Berm Distance");
    frc::SmartDashboard::PutNumber("Autonomy/Traversal/Starting Facing Direction", startingFacingDirection);
    frc::SmartDashboard::SetPersistent("Autonomy/Traversal/Starting Facing Direction");
    frc::SmartDashboard::PutNumber("Autonomy/Traversal/From Wall Distance", fromWallDistance);
    frc::SmartDashboard::SetPersistent("Autonomy/Traversal/From Wall Distance");

    frc::SmartDashboard::PutNumber("Autonomy/EXC Orient/Trench Index", trenchIndex);
    frc::SmartDashboard::PutNumber("Autonomy/EXC Orient/Trench Gap", trenchGap);
    frc::SmartDashboard::SetPersistent("Autonomy/EXC Orient/Trench Gap");
    frc::SmartDashboard::PutNumber("Autonomy/EXC Orient/Trench Start Dist", trenchStartDist);
    frc::SmartDashboard::SetPersistent("Autonomy/EXC Orient/Trench Start Dist");
    frc::SmartDashboard::PutNumber("Autonomy/EXC Orient/Failsafe Time", failsafeTime.value());
    frc::SmartDashboard::SetPersistent("Autonomy/EXC Orient/Failsafe Time");

    frc::SmartDashboard::PutNumber("Autonomy/EXC Action/Start Excavation Spin", startExcavationSpin);
    frc::SmartDashboard::SetPersistent("Autonomy/EXC Action/Start Excavation Spin");
    frc::SmartDashboard::PutNumber("Autonomy/EXC Action/Spin Time", excavationSpinTime.value());
    frc::SmartDashboard::SetPersistent("Autonomy/EXC Action/Spin Time");

    frc::SmartDashboard::PutNumber("Autonomy/DEP Orient/CrawlTime-MaxTime Factor", crawlTimeToMaxTimeFactor);
    frc::SmartDashboard::SetPersistent("Autonomy/DEP Orient/CrawlTime-MaxTime Factor");

    frc::SmartDashboard::PutNumber("Autonomy/DEP Action/Spin Time", hopperSpinTime.value());
    frc::SmartDashboard::SetPersistent("Autonomy/DEP Action/Spin Time");
}

void Autonomy::SetSystems(Vision* vision, Localization* localization, MobilitySubsystem* mob, ExcavationSubsystem* exc, HopperSubsystem* hop) {
    this->vision = vision;
    this->mob = mob;
    this->exc = exc;
    this->hop = hop;
    this->localization = localization;
}

void Autonomy::Init() {

    currentPhase = TRAVERSAL;
    this->pathing = new Pathfinding(vision, localization);
    pathing->Init();
    if (frc::SmartDashboard::GetBoolean("Autonomy/Phases/Traversal", false))
        TraversalInit();
}

bool Autonomy::Periodic() {

    switch(currentPhase) {
        case TRAVERSAL:
            // Navigate from Starting Zone using A* pathfinding and Localization system to a specific spot directly in front of the berm location.
            if (!frc::SmartDashboard::GetBoolean("Autonomy/Phases/Traversal", false) || (frc::SmartDashboard::GetBoolean("Autonomy/Phases/Traversal", false) && TraversalPeriodic())) {
                currentPhase = EXC_ORIENT;
                if (frc::SmartDashboard::GetBoolean("Autonomy/Phases/EXC Orient", false))
                    ExcOrientInit();
            }
            break;
        case EXC_ORIENT:
            // Based on which trench index we are going to, crab the wheels to the correct angle and drive until we reach the edge of the EXC zone.
            if (!frc::SmartDashboard::GetBoolean("Autonomy/Phases/EXC Orient", false) || (frc::SmartDashboard::GetBoolean("Autonomy/Phases/EXC Orient", false) && ExcOrientPeriodic())) {
                currentPhase = EXC_ACTION;
                if (frc::SmartDashboard::GetBoolean("Autonomy/Phases/EXC Action", false))
                    ExcActionInit();
            }
            break;
        case EXC_ACTION:
            // Plunge, EXC, and Crawl. Then backup the distance traveled during the crawl.
            if (!frc::SmartDashboard::GetBoolean("Autonomy/Phases/EXC Action", false) || (frc::SmartDashboard::GetBoolean("Autonomy/Phases/EXC Action", false) && ExcActionPeriodic())) {
                currentPhase = DEP_ORIENT;
                if (frc::SmartDashboard::GetBoolean("Autonomy/Phases/DEP Orient", false))
                    DepOrientInit();
            }
            break;
        case DEP_ORIENT:
            // Based on which trench index we just excavated, crab the wheels to the correct angle and drive until we reach the edge of the berm.
            if (!frc::SmartDashboard::GetBoolean("Autonomy/Phases/DEP Orient", false) || (frc::SmartDashboard::GetBoolean("Autonomy/Phases/DEP Orient", false) && DepOrientPeriodic())) {
                currentPhase = DEP_ACTION;
                if (frc::SmartDashboard::GetBoolean("Autonomy/Phases/DEP Action", false))
                    DepActionInit();
            }
            break;
        case DEP_ACTION:
            // Run the hopper belt to empty the system.
            if (!frc::SmartDashboard::GetBoolean("Autonomy/Phases/DEP Action", false) || (frc::SmartDashboard::GetBoolean("Autonomy/Phases/DEP Action", false) && DepActionPeriodic())) {
                if (frc::SmartDashboard::GetBoolean("Autonomy/Phases/Loop", false)) {
                    currentPhase = EXC_ORIENT;
                    if (frc::SmartDashboard::GetBoolean("Autonomy/Phases/EXC Orient", false))
                        ExcOrientInit();
                } else {
                    Kill();
                    return true;
                }
            }
            break;
    }

    return false;
}

bool finishedSteering = false;
bool skipTraversal;
bool readyToStraight = false;
bool readyToRotateToLineup = false;
bool readyToLineup = false;
bool readyToRotateToStraight = false;
bool readyToRotateToBerm = false;
bool readyToStraightToBerm = false;

void Autonomy::TraversalInit() {
    skipTraversal = !frc::SmartDashboard::GetBoolean("Autonomy/Phases/Traversal", false);
    if (skipTraversal) {
        fmt::print("Skipping Auto Traversal Init\n");
        return;
    }

    driveTimePerCell = frc::SmartDashboard::GetNumber("Autonomy/Traversal/DriveTimePerCell", driveTimePerCell);
    turnTimes[0] = units::time::second_t{frc::SmartDashboard::GetNumber("Autonomy/Traversal/TurnTime 45", turnTimes[0].value())};
    turnTimes[1] = units::time::second_t{frc::SmartDashboard::GetNumber("Autonomy/Traversal/TurnTime 90", turnTimes[1].value())};
    turnTimes[2] = units::time::second_t{frc::SmartDashboard::GetNumber("Autonomy/Traversal/TurnTime 135", turnTimes[2].value())};
    turnTimes[3] = units::time::second_t{frc::SmartDashboard::GetNumber("Autonomy/Traversal/TurnTime 180", turnTimes[3].value())};
    fromWallDistance = frc::SmartDashboard::GetNumber("Autonomy/Traversal/From Wall Distance", fromWallDistance);
    pathStartDistance = frc::SmartDashboard::GetNumber("Autonomy/Traversal/Path Start Distance", pathStartDistance);
    pathDistance = frc::SmartDashboard::GetNumber("Autonomy/Traversal/Path Distance", pathDistance);
    toBermDistance = frc::SmartDashboard::GetNumber("Autonomy/Traversal/To Berm Distance", toBermDistance);
    startingFacingDirection = frc::SmartDashboard::GetNumber("Autonomy/Traversal/Starting Facing Direction", startingFacingDirection);

    Blinkin::Set(BlinkinPattern::GREEN);

    readyToStraight = false;
    readyToLineup = false;
    timer.Stop();
    timer.Reset();
}
bool Autonomy::TraversalPeriodic() {
    if (skipTraversal) {
        fmt::print("Skipping Auto Traversal Periodic\n");
        return true;
    } 

    /////////////////////////////////
    //
    // This here was an attempt at a point and shoot type traversal autonomy. Not ideal, but we were trying for anything to get us more points.
    // This is not a recommended way of completing traversal autonomy in the future.
    //
    // This was written in the few days of our Universty of Central Florida runs and Kennedy Space Center.
    // Sadly even though nearly all of the parameters of movement were configurable, it was not enough to allow for
    // any points awarded in Autonmous Traversal for the AstroHuskies.
    //
    // Sorry,
    // Braeden Moore
    // 
    /////////////////////////////////

    if (readyToStraightToBerm) {
        finishedSteering = mob->SteerTo({0.0, 0.0, 0.0, 0.0});
        if (finishedSteering) {
            timer.Start();
            if (timer.HasElapsed(units::time::second_t{(toBermDistance - pathStartDistance) * 100 * driveTimePerCell})) {
                timer.Stop();
                timer.Reset();
                mob->StopAll();
                return true;
            } else {
                mob->Drive({-1.0, -1.0, -1.0, -1.0});
            }
        } else {
            mob->StopAll();
        }
    } else if (readyToRotateToBerm) {
        finishedSteering = mob->SteerTo({28.0, 28.0, 58.0, 58.0});
        if (finishedSteering) {
            timer.Start();
            if (timer.HasElapsed(turnTimes[1])) {
                timer.Stop();
                timer.Reset();
                mob->StopAll();
                readyToStraightToBerm = true;
            } else {
                mob->Drive({-0.55, 0.55, -1.0, 1.0});
            }
        } else {
            mob->StopAll();
        }
    } else if (readyToStraight) {
        finishedSteering = mob->SteerTo({0.0, 0.0, 0.0, 0.0});
        if (finishedSteering) {
            timer.Start();
            if (timer.HasElapsed(units::time::second_t{pathDistance * 100 * driveTimePerCell})) {
                timer.Stop();
                timer.Reset();
                mob->StopAll();
                readyToRotateToBerm = true;
            } else {
                mob->Drive({1.0, 1.0, 1.0, 1.0});
            }
        } else {
            mob->StopAll();
        }
    } else if (readyToRotateToStraight) {
        finishedSteering = mob->SteerTo({28.0, 28.0, 58.0, 58.0});
        if (finishedSteering) {
            timer.Start();
            if (timer.HasElapsed(turnTimes[1])) {
                timer.Stop();
                timer.Reset();
                mob->StopAll();
                readyToStraight = true;
            } else {
                if (startingFacingDirection == 2 || startingFacingDirection == 3) {
                    mob->Drive({0.55, -0.55, 1.0, -1.0});
                } else {
                    mob->Drive({-0.55, 0.55, -1.0, 1.0});
                }
            }
        } else {
            mob->StopAll();
        }
    } else if (readyToLineup) {
        finishedSteering = mob->SteerTo({0.0, 0.0, 0.0, 0.0});
        if (finishedSteering) {
            timer.Start();
            if (timer.HasElapsed(units::time::second_t{pathStartDistance * 100 * driveTimePerCell})) {
                timer.Stop();
                timer.Reset();
                mob->StopAll();
                readyToRotateToStraight = true;
            } else {
                if (startingFacingDirection == 0 || startingFacingDirection == 3) { // Driving Forwards
                    mob->Drive({1.0, 1.0, 1.0, 1.0});
                } else { // Driving Backwards
                    mob->Drive({-1.0, -1.0, -1.0, -1.0});
                }
            }

        } else {
            mob->StopAll();
        }
    } else if (readyToRotateToLineup) {
        if (pathStartDistance >= 0.5) {
            if (startingFacingDirection == 0 || startingFacingDirection == 2) {
                finishedSteering = mob->SteerTo({28.0, 28.0, 58.0, 58.0 });
                if (finishedSteering) {
                    timer.Start();
                    if (timer.HasElapsed(turnTimes[1])) {
                        timer.Stop();
                        timer.Reset();
                        mob->StopAll();
                        readyToLineup = true;
                    } else {
                        if (startingFacingDirection == 2)
                            mob->Drive({0.55, -0.55, 1.0, -1.0});
                        else    
                            mob->Drive({-0.55, 0.55, -1.0, 1.0});
                    }
                } else {
                    mob->StopAll();
                }
            } else {
                readyToLineup = true;
            }
        } else {
            if (startingFacingDirection == 1 || startingFacingDirection == 3) {
                finishedSteering = mob->SteerTo({28.0, 28.0, 58.0, 58.0 });
                if (finishedSteering) {
                    timer.Start();
                    if (timer.HasElapsed(turnTimes[1])) {
                        timer.Stop();
                        timer.Reset();
                        mob->StopAll();
                        readyToLineup = true;
                    } else {
                        if (startingFacingDirection == 1)
                            mob->Drive({0.55, -0.55, 1.0, -1.0});
                        else    
                            mob->Drive({-0.55, 0.55, -1.0, 1.0});
                    }
                } else {
                    mob->StopAll();
                }
            } else {
                readyToLineup = true;
            }
        }
    } else {
        if (startingFacingDirection == 0 || startingFacingDirection == 2)
            finishedSteering = mob->SteerTo({-22.0, 22.0, 22.0, -22.0});
        else
            finishedSteering = mob->SteerTo({22.0, -22.0, -22.0, 22.0});
        if (finishedSteering) {
            timer.Start();
            if (timer.HasElapsed(units::time::second_t{fromWallDistance * 100 * driveTimePerCell})) {
                timer.Stop();
                timer.Reset();
                mob->StopAll();
                readyToRotateToLineup = true;
            } else {
                if (startingFacingDirection == 0 || startingFacingDirection == 3)
                        mob->Drive({1.0, 1.0, 1.0, 1.0});
                    else
                        mob->Drive({-1.0, -1.0, -1.0, -1.0});
            }
        } else {
            mob->StopAll();
        }
    }
    
    /////////////////////////////////
    // 
    // The next comment chunk was all an attempt to implement the Pathfinding system that was written in the few weeks after the end of the semester.
    // Unfortunately the team never saw this implementation work. All of the theory and simulations of the algorithm were written during those few
    // weeks between end of semester, the simulation can be found in a zipped folder in the RAN google drive.
    // 
    // While the simulation was running beautifully, integration into this rover code wasn't started until the actual drive from Michigan to Florida.
    // The thing I couldn't quite figure out was how to update the path occasionally when new obstacles became visible,
    // This meant either you keep running the new path while the new one is calcuating, updating the path when it finishes
    // Or you stop and hold in place while calculating, which would take much longer, especially when each new turn may introduce a new obstacle.
    // 
    // Another small problem to learn how to solve is that the function needs to be asyncronous, understanding how an async function works and how
    // to query the results from the async function is important to balance the calculation time with the execution time.
    //
    // I wish you luck,
    // Braeden Moore
    //
    /////////////////////////////////

    /*

    pathing->Periodic();

    newNode = pathing->queryPath();

    if (newNode) { // newPath is not a nullptr
        if (newNode->motionType == MobilitySubsystem::MobilityMode::FREE) { // Still calculating path, should stop and wait.
            mob->StopAll();
            return false;
        } else { // Path was calculated successfully
            prevNode = currNode;
            currNode = newNode;
        }
    } else { // Either pathfinding algorithm timed out or it couldn't find a path
        
        // Should handle the couldn't find a path case in the algorithm itself, allowing passage over the smallest rock that was hit a lot.
        
        // If the algorithm itself timed out, we need to think of another plan... 
        // I don't really know what the alternative is.
        // Drive straight toward target...?

        return false;
    }

    std::array<double, 4> speedFactors = {1.0, 1.0, 1.0, 1.0};

    if (currNode->motionType == MobilitySubsystem::MobilityMode::ACKERMANN) {
        double angle = fabs(currNode->motionData) / 180.0 * M_PI;
        double centerOffset = (40.0 + (181.0 / 6.0) * tan(angle)) / tan(angle);
        centerOffset = std::max(106.0, std::min(centerOffset, 406.0));
        double closeAngle = atan(40.0 / (centerOffset - (181.0 / 6.0))) * 180.0 / M_PI;
        double farAngle = atan(40.0 / (centerOffset + (181.0 / 6.0))) * 180.0 / M_PI;
        closeAngle = std::max(-28.0, std::min(closeAngle, 28.0));
        farAngle = std::max(-28.0, std::min(farAngle, 28.0));

        double factor = cos((closeAngle - farAngle) / 180.0 * M_PI);

        std::array<double, 4> steerAngles = {0.0, 0.0, 0.0, 0.0};

        if (currNode->motionData > 0.0) { // Steer right
            steerAngles[0] = farAngle;
            steerAngles[1] = -closeAngle;
            steerAngles[2] = farAngle;
            steerAngles[3] = -closeAngle;
            speedFactors = {1.0, factor, 1.0, factor};
        } else if (currNode->motionData < 0.0) { // Steer left
            steerAngles[0] = -closeAngle;
            steerAngles[1] = farAngle;
            steerAngles[2] = -closeAngle;
            steerAngles[3] = farAngle;
            speedFactors = {factor, 1.0, factor, 1.0};
        }
        finishedSteering = mob->SteerTo(steerAngles);
    } else if (currNode->motionType == MobilitySubsystem::MobilityMode::CRAB) {
        finishedSteering = mob->SteerTo({currNode->motionData, -currNode->motionData, -currNode->motionData, currNode->motionData});
    } else if (currNode->motionType == MobilitySubsystem::MobilityMode::ZERO_POINT) {
        speedFactors = {0.55, 0.55, 1.0, 1.0};
        finishedSteering = mob->SteerTo({28.0, 28.0, 58.0, 58.0});
    } else {
        // Something went slightly wrong, our pathfinding shouldn't be using crawl, or free movement.
        wpi::errs() << "Error in putting path into action, found mobilityType that was not expected";   
    }

    if (currNode->reverse) {
        speedFactors[0] *= -1;
        speedFactors[1] *= -1;
        speedFactors[2] *= -1;
        speedFactors[3] *= -1;
    }

    if (!finishedSteering) {
        mob->StopAll();
        return false;
    }

    units::time::second_t driveTime = (currNode->motionType == MobilitySubsystem::MobilityMode::ZERO_POINT) ? (turnTimes[0] / 45.0 * currNode->motionData) : units::time::second_t{driveTimePerCell};
    timer.Start();
    if (timer.HasElapsed(driveTime)) {
        prevNode = currNode;
        currNode = currNode->parent;
        if (pathing->reachedTarget(currNode))
            finalTurn = true;            
        timer.Stop();
        timer.Reset();
    } else {
        mob->Drive(speedFactors);
    }
    */

    /////////////////////////////////
    //
    // This was the very first attempt at autonomy by the RAN team, this utilized a basic A* pathfinding algorithm
    // Unfortunately the A* algorithm itself is not longer here, it used to be located in the Localization system.
    // This algorithm did not take into account any obstacles, so was not that great for actually solving the problem
    // of traversal. This was really just a proof of concept especially because it was really slow, using only
    // straight movements and 90 degree turns using the Zero Point turn to navigate around the arena with 10 centimeter cells.
    //
    // This is not really recommended for a few reasons, one, even if obstacles were introduced into the algorithm, the execution
    // of using zero point turn to move between neighboring cells is very inefficient on power and time.
    //
    // Good luck,
    // Braeden Moore
    // 
    /////////////////////////////////

    /*
    if (currPathStep == 0) {
        
        currPathStep++;
        a = path[currPathStep - 1];
        b = path[currPathStep];
        globalDir = 0;
        
        if      (b.x == a.x && b.y >= a.y) globalDir = 0;
        else if (b.x > a.x  && b.y > a.y)  globalDir = 45;
        else if (b.x > a.x  && b.y == a.y) globalDir = 90;
        else if (b.x > a.x  && b.y < a.y)  globalDir = 135;
        else if (b.x == a.x && b.y < a.y)  globalDir = 180;
        else if (b.x < a.x  && b.y < a.y)  globalDir = 225;
        else if (b.x < a.x  && b.y == a.y) globalDir = 270;
        else if (b.x < a.x  && b.y > a.y)  globalDir = 315;
    }

    offDir = (globalDir - pseudoRoverDir + 360) % 360;

    // Not facing towards the next point in the path.
    if (offDir != 0) {
        // Steer Wheels to Zero Point Configuration
        if (!mob->SteerTo({28.0, 28.0, 58.0, 58.0,})) {
            mob->StopAll();
            return false;
        }

        timer.Start();
        if (offDir <= 180) {
            mob->Drive({0.55, -0.55, 1.0, -1.0});
            if (timer.HasElapsed(turnTimes[int(offDir / 45) - 1])) {
                timer.Stop();
                timer.Reset();
                pseudoRoverDir = globalDir;
            }
        } else {
            mob->Drive({-0.55, 0.55, -1.0, 1.0});
            if (timer.HasElapsed(turnTimes[int((360 - offDir) / 45) - 1])) {
                timer.Stop();
                timer.Reset();
                pseudoRoverDir = globalDir;
            }
        }
        
    } else {

        if (!mob->SteerTo({0.0, 0.0, 0.0, 0.0})) {
            mob->StopAll();
            return false;
        }

        if (currPathStep == int(path.size()))
            return true;

        double diagonalFactor = pseudoRoverDir % 90 == 0 ? 1.0 : std::sqrt(2.0);
        timer.Start();
        mob->Drive({1.0, 1.0, 1.0, 1.0});
        if (timer.HasElapsed(units::time::second_t{driveTimePerCell * diagonalFactor})) {
            timer.Stop();
            timer.Reset();
            currPathStep++;

            if (currPathStep == int(path.size())) {
                mob->StopAll();
                globalDir = 0;
                return false;
            }

            a = path[currPathStep - 1];
            b = path[currPathStep];
            globalDir = 0;
            
            if      (b.x == a.x && b.y >= a.y) globalDir = 0;
            else if (b.x > a.x  && b.y > a.y)  globalDir = 45;
            else if (b.x > a.x  && b.y == a.y) globalDir = 90;
            else if (b.x > a.x  && b.y < a.y)  globalDir = 135;
            else if (b.x == a.x && b.y < a.y)  globalDir = 180;
            else if (b.x < a.x  && b.y < a.y)  globalDir = 225;
            else if (b.x < a.x  && b.y == a.y) globalDir = 270;
            else if (b.x < a.x  && b.y > a.y)  globalDir = 315;
        }
    }
    */

    return false;
}

/////////////////////////////////////
//
// The rest of the autonomy code for Excavation and Deposition worked enough to get us a single successful cycle in day 1 of KSC,
// and three successfull back to back cycles in day 2 of KSC. This is also the code that awarded the Michigan Tech AstroHuskies
// 4th place for the Autonomy Award. This was a very simple implementation that was purely time based for movements.
//
// The largest recommendation from me to you is to use some form of localization to know where the rover actually is positioned, in conjunction
// with the onboard NavX IMU to update the current position of the rover. With this data, it could prevented any of the issues we had where we
// ran into the backwall a few times or drifted off the of the berm after a few cycles. All of these problems were because of the fact it was
// time based.
//
// This implementation also never took into account obstacles in the path of the excavation, this is nearly required if you want to go for
// a larger amount of autonomous excavation cycles. It may be difficult though as visibility of any rocks you may hit are obstructed during excavation
// due to the dust and shaking of the rover during operation.
//
// You can do this,
// Braeden Moore
//
/////////////////////////////////////

bool skipExcOrient = false;
double crabAngle = 0.0;
Coord posStart{0.0, 0.0};
units::time::second_t alignTime{0.0};
bool alignMode = true;

void Autonomy::ExcOrientInit() { 
    skipExcOrient = !frc::SmartDashboard::GetBoolean("Autonomy/Phases/EXC Orient", false);
    if (skipExcOrient) {
        fmt::print("Skipping Auto EXC Orient Init\n");
        return;
    }
    
    Blinkin::Set(BlinkinPattern::RED);

    trenchGap = frc::SmartDashboard::GetNumber("Autonomy/EXC Orient/Trench Gap", trenchGap);
    trenchStartDist = frc::SmartDashboard::GetNumber("Autonomy/EXC Orient/Trench Start Dist", trenchStartDist);
    failsafeTime = units::time::second_t{frc::SmartDashboard::GetNumber("Autonomy/EXC Orient/Failsafe Time", failsafeTime.value())};

    trenchIndex = (int(frc::SmartDashboard::GetNumber("Autonomy/EXC Orient/Trench Index", trenchIndex))) % calcTrenchCount();
    frc::SmartDashboard::PutNumber("Autonomy/EXC Orient/Trench Index", trenchIndex);

    crabAngle = calcTrenchAngle();

    alignMode = true;
    timer.Stop();
    timer.Reset();
    finishedSteering = false;
}
bool Autonomy::ExcOrientPeriodic() {
    if (skipExcOrient) {
        fmt::print("Skipping Auto EXC Orient Periodic\n");
        return true;
    }

    if (alignMode) {
        finishedSteering = mob->SteerTo({crabAngle, -crabAngle, -crabAngle, crabAngle});
        if (finishedSteering) {
            timer.Start();
            if (timer.HasElapsed(failsafeTime)) {
                timer.Stop();
                timer.Reset();
                alignMode = false;
            } else {
                mob->Drive({1.0, 1.0, 1.0, 1.0});
            }
        } else {
            mob->StopAll();
        }
    } else {
        finishedSteering = mob->SteerTo({0.0, 0.0, 0.0, 0.0});
        if (finishedSteering) {
            timer.Start();
            if (timer.HasElapsed(alignTime)) {
                timer.Stop();
                timer.Reset();
                mob->StopAll();
                return true;
            } else {
                mob->Drive({1.0, 1.0, 1.0, 1.0});
            }
        } else {
            mob->StopAll();
        }
    }
    return false;
}

bool skipExcAction = false;

void Autonomy::ExcActionInit() {
    skipExcAction = !frc::SmartDashboard::GetBoolean("Autonomy/Phases/EXC Action", false);
    if (skipExcAction) {
        fmt::print("Skipping Auto EXC Action Init\n");
        return;
    }

    Blinkin::Set(BlinkinPattern::HEARTBEAT_RED);

    startExcavationSpin = frc::SmartDashboard::GetNumber("Autonomy/EXC Action/Start Excavation Spin", startExcavationSpin);
    excavationSpinTime = units::time::second_t{frc::SmartDashboard::GetNumber("Autonomy/EXC Action/Spin Time", excavationSpinTime.value())};

    timer.Stop();
    timer.Reset();
    crawlTimer.Stop();
    crawlTimer.Reset();
    finishedSteering = false;
}
bool Autonomy::ExcActionPeriodic() {
    if (skipExcAction) {
        fmt::print("Skipping Auto EXC Action Periodic\n");
        return true;
    }

    finishedSteering = mob->SteerTo({0.0, 0.0, 0.0, 0.0});
    if (finishedSteering) {
        if (timer.HasElapsed(excavationSpinTime)) {
            mob->StopAll();
            crawlTimer.Stop();
            exc->StartActuate(true);
            if (exc->GetActuateLower() < startExcavationSpin) {
                exc->Spin(0.0, false);
                hop->Stop();
                if (exc->GetSpeed() < 0.01) {
                    exc->Stop();
                    return true;
                }
            }
        } else {
            exc->StartActuate(false);
            if (exc->GetActuateLower() >= startExcavationSpin) {
                timer.Start();
                exc->Spin(1.0, false);
                hop->Spin(1.0, false, true);
                if (exc->GetActuateUpper() >= exc->actuateMax) {
                    mob->Crawl(true);          
                    crawlTimer.Start();
                }
            }
        }
    }

    return false;
}

bool skipDepOrient = false;
units::time::second_t undoCrawlTime{0.0};

void Autonomy::DepOrientInit() {
    skipDepOrient = !frc::SmartDashboard::GetBoolean("Autonomy/Phases/DEP Orient", false);
    if (skipDepOrient) {
        fmt::print("Skipping Auto DEP Orient Init\n");
        return;
    }

    Blinkin::Set(BlinkinPattern::BLUE);

    trenchGap = frc::SmartDashboard::GetNumber("Autonomy/EXC Orient/Trench Gap", trenchGap);
    trenchStartDist = frc::SmartDashboard::GetNumber("Autonomy/EXC Orient/Trench Start Dist", trenchStartDist);
    trenchIndex = int(frc::SmartDashboard::GetNumber("Autonomy/EXC Orient/Trench Index", trenchIndex)) % calcTrenchCount();
    crabAngle = calcTrenchAngle();

    failsafeTime = units::time::second_t{frc::SmartDashboard::GetNumber("Autonomy/EXC Orient/Failsafe Time", failsafeTime.value())};

    crawlTimeToMaxTimeFactor = frc::SmartDashboard::GetNumber("Autonomy/DEP Orient/CrawlTime-MaxTime Factor", crawlTimeToMaxTimeFactor);
    undoCrawlTime = units::time::second_t{crawlTimer.Get().value() * crawlTimeToMaxTimeFactor};    

    timer.Stop();
    timer.Reset();
    crawlTimer.Stop();
    crawlTimer.Reset();
    finishedSteering = false;
}
bool Autonomy::DepOrientPeriodic() {
    if (skipDepOrient) {
        fmt::print("Skipping Auto DEP Orient Periodic\n");
        return true;
    }

    if (alignTime.value() > 0.0 || undoCrawlTime.value() > 0.0)
        crawlTimer.Start();
        
    if (!crawlTimer.HasElapsed(undoCrawlTime + alignTime)) {
        finishedSteering = mob->SteerTo({0.0, 0.0, 0.0, 0.0});
        if (finishedSteering)
            mob->Drive({-1.0, -1.0, -1.0, -1.0});
        else
            mob->StopAll();
    } else {
        crawlTimer.Stop();
        finishedSteering = mob->SteerTo({crabAngle, -crabAngle, -crabAngle, crabAngle});
        if (finishedSteering) {
            timer.Start();
            if (timer.HasElapsed(failsafeTime)) {
                mob->StopAll();
                return true;
            } else {
                mob->Drive({-1.0, -1.0, -1.0, -1.0});
            }
        } else {
            mob->StopAll();
        }
    }
    return false;
}

bool skipDepAction = false;

void Autonomy::DepActionInit() {
    skipDepAction = !frc::SmartDashboard::GetBoolean("Autonomy/Phases/DEP Action", false);
    if (skipDepAction) {
        fmt::print("Skipping Auto DEP Action Init\n");
        return;
    }

    Blinkin::Set(BlinkinPattern::HEARTBEAT_BLUE);

    alignTime = units::time::second_t{alignTime.value() + 0.5};
    hopperSpinTime = units::time::second_t{frc::SmartDashboard::GetNumber("Autonomy/DEP Action/Spin Time", hopperSpinTime.value())};

    timer.Stop();
    timer.Reset();
}
bool Autonomy::DepActionPeriodic() {
    if (skipDepAction) {
        fmt::print("Skipping Auto DEP Action Periodic\n");
        return true;
    }
    
    timer.Start();
    hop->Spin(1.0, false, false);
    if (timer.HasElapsed(hopperSpinTime)) {
        timer.Stop();
        hop->Stop();
        return true;
    }
    return false;
}

/////////////////////////////////
// Autonomous Helper Functions //
/////////////////////////////////

void Autonomy::Kill() {
    wpi::outs() << "Kill Command Issued\n";
    frc2::CommandScheduler::GetInstance().CancelAll();
    mob->Reset();
    exc->Reset();
    hop->Reset();
}

int Autonomy::calcTrenchCount() {
    return abs(int(tan(26.0 / 180.0 * M_PI) * trenchStartDist / trenchGap)) * 2 + 1;
}

double Autonomy::calcTrenchAngle() {
    return atan(((trenchIndex - (calcTrenchCount() - 1) / 2) * trenchGap) / trenchStartDist) * 180.0 / M_PI;
}