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
    
    frc::SmartDashboard::PutNumber("Autonomy/Traversal/Target X", targetPos.x);
    frc::SmartDashboard::PutNumber("Autonomy/Traversal/Target Y", targetPos.y);
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

    frc::SmartDashboard::PutNumber("Autonomy/EXC Orient/Trench Index", trenchIndex);
    frc::SmartDashboard::PutNumber("Autonomy/EXC Orient/Trench Gap", trenchGap);
    frc::SmartDashboard::SetPersistent("Autonomy/EXC Orient/Trench Gap");
    frc::SmartDashboard::PutNumber("Autonomy/EXC Orient/Trench Start Dist", trenchStartDist);
    frc::SmartDashboard::SetPersistent("Autonomy/EXC Orient/Trench Start Dist");
    frc::SmartDashboard::PutNumber("Autonomy/EXC Orient/Failsafe Time", failsafeTime.value());
    frc::SmartDashboard::SetPersistent("Autonomy/EXC Orient/Failsafe Time");

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

bool skipTraversal;
std::vector<Coord> path;
int currPathStep = 0;
int pseudoRoverDir = 0;
int globalDir = 0;
int offDir = 0;
Coord a{0.0, 0.0};
Coord b{0.0, 0.0};

void Autonomy::TraversalInit() {
    skipTraversal = !frc::SmartDashboard::GetBoolean("Autonomy/Phases/Traversal", false);
    if (skipTraversal) {
        fmt::print("Skipping Auto Traversal Init\n");
        return;
    }

    targetPos.x = frc::SmartDashboard::GetNumber("Autonomy/Traversal/Target X", targetPos.x);
    targetPos.y = frc::SmartDashboard::GetNumber("Autonomy/Traversal/Target Y", targetPos.y);
    driveTimePerCell = frc::SmartDashboard::GetNumber("Autonomy/Traversal/DriveTimePerCell", driveTimePerCell);
    turnTimes[0] = units::time::second_t{frc::SmartDashboard::GetNumber("Autonomy/Traversal/TurnTime 45", turnTimes[0].value())};
    turnTimes[1] = units::time::second_t{frc::SmartDashboard::GetNumber("Autonomy/Traversal/TurnTime 90", turnTimes[1].value())};
    turnTimes[2] = units::time::second_t{frc::SmartDashboard::GetNumber("Autonomy/Traversal/TurnTime 135", turnTimes[2].value())};
    turnTimes[3] = units::time::second_t{frc::SmartDashboard::GetNumber("Autonomy/Traversal/TurnTime 180", turnTimes[3].value())};
    double obsX = frc::SmartDashboard::GetNumber("Localization/Obstacle X", -1);
    double obsY = frc::SmartDashboard::GetNumber("Localization/Obstacle Y", -1);
    double obsR = frc::SmartDashboard::GetNumber("Localization/Obstacle R", 0);

    localization->clearGrid();
    if (obsX != -1 && obsY != -1)
        localization->addObstacle(Coord{obsX, obsY}, obsR);

    path = localization->findPath(targetPos);    
    wpi::outs() << "\n\nA* Path to " << targetPos.toStr() << " with no obstacles:\n";
    std::string msg = "";
    for (Coord point : path)
        msg = msg + " " + point.toStr() + "\n";
    wpi::outs() << msg << "\n Path has " << std::to_string(path.size()) << " nodes.\n\n";
    frc::SmartDashboard::PutString("Localization/Display", localization->displayGrid(targetPos, path));

    Blinkin::Set(BlinkinPattern::GREEN);

    currPathStep = 0;
    pseudoRoverDir = 0;
    timer.Stop();
    timer.Reset();
}
bool Autonomy::TraversalPeriodic() {
    if (skipTraversal) {
        fmt::print("Skipping Auto Traversal Periodic\n");
        return true;
    } 

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

    return false;
}

bool skipExcOrient = false;
bool finishedSteering = false;
double crabAngle = 0.0;
Coord posStart{0.0, 0.0};

void Autonomy::ExcOrientInit() { 
    skipExcOrient = !frc::SmartDashboard::GetBoolean("Autonomy/Phases/EXC Orient", false);
    if (skipExcOrient) {
        fmt::print("Skipping Auto EXC Orient Init\n");
        return;
    }
    
    Blinkin::Set(BlinkinPattern::RED);

    trenchGap = frc::SmartDashboard::GetNumber("Autonomy/EXC Orient/Trench Gap", trenchGap);
    trenchStartDist = frc::SmartDashboard::GetNumber("Autonomy/EXC Orient/Trench Start Dist", trenchStartDist);
    trenchIndex = (int(frc::SmartDashboard::GetNumber("Autonomy/EXC Orient/Trench Index", trenchIndex)) + 1) % calcTrenchCount();
    frc::SmartDashboard::PutNumber("Autonomy/EXC Orient/Trench Index", trenchIndex);

    failsafeTime = units::time::second_t{frc::SmartDashboard::GetNumber("Autonomy/EXC Orient/Failsafe Time", failsafeTime.value())};

    crabAngle = calcTrenchAngle();
    posStart = localization->getRoverCenter();

    timer.Stop();
    timer.Reset();
    finishedSteering = false;
}
bool Autonomy::ExcOrientPeriodic() {
    if (skipExcOrient) {
        fmt::print("Skipping Auto EXC Orient Periodic\n");
        return true;
    }

    finishedSteering = mob->SteerTo({crabAngle, -crabAngle, -crabAngle, crabAngle});
    if (finishedSteering) {
        timer.Start();
        if ((posStart.y + trenchStartDist) < localization->getRoverCenter().y || timer.HasElapsed(failsafeTime)) {
            mob->StopAll();
            return true;
        } else {
            mob->Drive({1.0, 1.0, 1.0, 1.0});
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
            if (!exc->canSpin()) {
                exc->Spin(0.0, false);
                hop->Stop();
                if (exc->GetSpeed() < 0.01) {
                    exc->Stop();
                    return true;
                }
            }
        } else {
            exc->StartActuate(false);
            if (exc->canSpin()) {
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

    if (undoCrawlTime.value() > 0.0)
        crawlTimer.Start();
        
    if (localization->getRoverCenter().y > (posStart.y + trenchStartDist) && !crawlTimer.HasElapsed(undoCrawlTime)) {
        crawlTimer.Stop();
        finishedSteering = mob->SteerTo({0.0, 0.0, 0.0, 0.0});
        if (finishedSteering)
            mob->Drive({-1.0, -1.0, -1.0, -1.0});
    } else {
        finishedSteering = mob->SteerTo({crabAngle, -crabAngle, -crabAngle, crabAngle});
        if (finishedSteering) {
            timer.Start();
            if (localization->getRoverCenter().y < posStart.y || timer.HasElapsed(failsafeTime)) {
                mob->StopAll();
                return true;
            } else {
                mob->Drive({-1.0, -1.0, -1.0, -1.0});
            }
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