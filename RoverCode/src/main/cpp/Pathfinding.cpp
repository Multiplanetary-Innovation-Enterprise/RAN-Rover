#include <Pathfinding.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include <math.h>
#include <cmath>
#include <future>
#include <iostream>
#include <thread>

Pathfinding::Pathfinding(Vision* vision, Localization* local): vision(vision), local(local) {
    frc::SmartDashboard::PutNumber("Pathfinding/Recalc Period", recalcPeriod.value());
    frc::SmartDashboard::SetPersistent("Pathfinding/Recalc Period");
    frc::SmartDashboard::PutNumber("Pathfinding/Read Objects Period", readObjectsPeriod.value());
    frc::SmartDashboard::SetPersistent("Pathfinding/Read Objects Period");
}

void Pathfinding::Init() {
    objects.clear();
    recalcTimer.Reset();
}

void Pathfinding::Periodic() {
    
    readObjectsPeriod = units::time::second_t{frc::SmartDashboard::GetNumber("Pathfinding/Read Objects Period", readObjectsPeriod.value())};
    if (readObjectsTimer.HasElapsed(readObjectsPeriod)) {
        std::vector<Object> objs = vision->IdentifyObjects();

        for (Object obj : objs) {
            double angleToObject = std::abs(obj.maxAngle - obj.minAngle);
            double objectRadius = obj.dist * std::tan((angleToObject / 180.0 * M_PI));
            double globalAngleToObject = angleToObject + local->getRoverAngle();
            double deltaX = std::sin(globalAngleToObject);
            double deltaY = std::cos(globalAngleToObject);
            addObject(local->getRoverCenter() + Coord{deltaX, deltaY}, objectRadius);
        }
    }

    // Could be heavily improved by testing the already created path against any new obstacles, assuming the target remains the same.
    // This helps to prevent a ton of new calculations if the first half of the path can remain untouched for example.
    // To be done in future...

    // I really don't know if this asyncronous running will work on the roboRio nicely, yet to test it... whoops.
    std::future_status pathStatus = path.wait_for(std::chrono::seconds(0));
    if (pathStatus == std::future_status::ready || pathStatus == std::future_status::timeout) {
        recalcPeriod = units::time::second_t{frc::SmartDashboard::GetNumber("Pathfinding/Recalc Period", recalcPeriod.value())};
        recalcTimer.Start();
        if (recalcTimer.HasElapsed(recalcPeriod)) {
            recalcTimer.Stop();
            recalcTimer.Reset();
            std::tuple<Coord, double> t = local->getDepositionTarget();
            setTarget(std::get<0>(t), std::get<1>(t));
            path = std::async(std::launch::async, [this]{ return findPath(local->getRoverCenter(), local->getRoverAngle()); });
        }
    }
}

Node* Pathfinding::findPath(Coord startPos, double startAngle) {
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> frontierSet;
    std::unordered_set<std::string> visitedSet;

    double angleToTarget = std::abs(std::atan2(targetPos.y - startPos.y, targetPos.x - startPos.x) - startAngle);
    double estimate = ((startPos.dist(targetPos) * 0.3) + angleToTarget) * 0.4;
    Node* start = new Node(startPos, startAngle, 0.0, estimate, MobilitySubsystem::MobilityMode::ACKERMANN, 0.0, false);
    frontierSet.push(start);

    while (!frontierSet.empty()) {
        Node* curr = frontierSet.top();
        frontierSet.pop();
        visitedSet.insert(makeVisitedKey(curr));

        if(reachedTarget(curr)) {
            return curr;
            hasQueried = false;
        }

        // Update Neighbors
        for (Node* next : generateSuccessors(curr)) {
            std::string visitedKey = makeVisitedKey(next);
            if (visitedSet.find(visitedKey) != visitedSet.end()) { // Already visited here before
                delete next;
                continue;
            }

            if (!isValid(next)) { // Collision detected
                visitedSet.insert(visitedKey);
                delete next;
                continue;
            }

            // Could be slightly more efficient if we can replace nodes that are in the same spot and same angle with a lower cost instead of pushing an additional node.
            frontierSet.push(next);
        }
    }

    hasQueried = false;
    return nullptr;
}

Node* Pathfinding::queryPath() {
    if (!hasQueried) {
        hasQueried = true;
        switch (path.wait_for(std::chrono::seconds(0))) {
            case std::future_status::ready:
                return path.get();
            case std::future_status::timeout:
                return nullptr;
            case std::future_status::deferred:
                return new Node(local->getRoverCenter(), local->getRoverAngle(), 0.0, 0.0, MobilitySubsystem::MobilityMode::FREE, 0.0, false);
        }
    }
    return nullptr;
}

void Pathfinding::setTarget(Coord pos, double angle) {
    this->targetPos = pos;
    this->targetAngle = angle;
}

//////////////////////////////
// Private Helper Functions //
//////////////////////////////

// Since the goal reached condition only cares it is in position, not facing the right direction, we will almost always need to zero point turn at the end to align ourselves.
bool Pathfinding::reachedTarget(Node* node) {
    return node->dist(targetPos) < PathingConstants::targetTolerance;
}

bool Pathfinding::isValid(Node* node) {
    // Calculate rover buffer
    Coord roverPos = local->getRoverCenter();
    Coord roverSize = PathingConstants::roverSize;
    Coord arenaSize = local->getArenaSize();
    double minRoverBuffer = roverSize.x / 2.0;
    double maxRoverBuffer = std::hypot(roverSize.x, roverSize.y) / 2.0;
    
    double globalWidth = roverSize.x * std::sin((local->getRoverAngle() / 180.0 * M_PI));
    double globalHeight = roverSize.y * std::cos((local->getRoverAngle() / 180.0 * M_PI));

    // Would hit the walls
    if (roverPos.x - (globalWidth / 2) < 10.0 || roverPos.x + (globalWidth / 2) > arenaSize.x - 10.0
     || roverPos.y - (globalHeight / 2) < 10.0 || roverPos.y + (globalHeight / 2) > arenaSize.y - 10.0)
        return false;

    for (int i = 0; i < int(objects.size()); i++) {
        Coord pos = std::get<0>(objects[i]);
        double radius = std::get<1>(objects[i]);
        double dist = std::hypot(pos.x - roverPos.x, pos.y - roverPos.y);
        
        if (std::get<2>(objects[i]) == -1)
            continue;

        if (dist + radius > maxRoverBuffer + 10.0)
            continue;
        
        if (dist + radius < minRoverBuffer) {
            std::get<2>(objects[i])++;
            return false;
        }

        double angleToObject = std::tan(pos.x - roverPos.x / pos.y - roverPos.y) - local->getRoverAngle();
        double angleOfLongestEdge = std::atan(roverSize.y / roverSize.x);
        double buffer = minRoverBuffer;

        if (angleToObject < angleOfLongestEdge) {
            double side = (roverSize.y / 2) * std::tan(angleToObject);
            buffer = std::hypot(roverSize.y / 2, side);
        } else {
            double side = (roverSize.x / 2) * std::tan(angleToObject);
            buffer = std::hypot(roverSize.x / 2, side);
        }

        if (dist + radius < buffer) {
            std::get<2>(objects[i])++;
            return false;
        }
    }

    return true;
}

std::string Pathfinding::makeVisitedKey(Node* node) {
    int x = static_cast<int>(std::round(node->x / PathingConstants::gridResolution));
    int y = static_cast<int>(std::round(node->y / PathingConstants::gridResolution));
    int a = static_cast<int>(std::round(node->angle / 10.0));
    return std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(a);
}

double Pathfinding::calculateSteerCost(MobilitySubsystem::MobilityMode currMode, double currSteer, MobilitySubsystem::MobilityMode nextMode, double nextSteer) {
    
    float cost = 0.0f;

    if (currMode == nextMode) {
        if (currMode != MobilitySubsystem::MobilityMode::ZERO_POINT)
            cost = std::abs(currSteer - nextSteer) / (28.0f / 3.0f);
    } else {
        if (currMode == MobilitySubsystem::MobilityMode::ZERO_POINT)
            cost = (58.f + std::abs(nextSteer)) / (58.0f / 8.0f);
        else if (nextMode == MobilitySubsystem::MobilityMode::ZERO_POINT)
            cost = (58.f + std::abs(currSteer)) / (58.0f / 8.0f);
        else
            cost = (std::abs(currSteer) + std::abs(nextSteer)) / (28.0f / 3.0f);
    }
    
    return cost + 1.0f;
}

std::vector<Node*> Pathfinding::generateSuccessors(Node* curr) {
    std::vector<Node*> successors;

    // Ackermann
    for (double steerAngle : {0.0, 5.0, -5.0, 10.0, -10.0, 15.0, -15.0, 20.0, -20.0, 25.0, -25.0}) {
        for (int reverse = 0; reverse <= 1; reverse++) {
            double step = reverse ? -PathingConstants::stepSize : PathingConstants::stepSize;
            
            double x = curr->x + (step * std::cos(curr->angle));
            double y = curr->y + (step * std::sin(curr->angle));
            double turnRadius = (PathingConstants::roverSize.y / 2.0) / std::tan(steerAngle / 180.0 * M_PI) + (PathingConstants::roverSize.x / 2.0);
            double a = curr->angle + (step / turnRadius);

            double cost = curr->cost + calculateSteerCost(curr->motionType, curr->motionData, MobilitySubsystem::MobilityMode::ACKERMANN, steerAngle);
            double angleToTarget = std::abs(std::atan2(targetPos.y - y, targetPos.x - x) - a);
            double estimate = ((curr->dist(targetPos) * 0.3) + angleToTarget) * 0.4;

            successors.push_back(new Node(x, y, a, cost, estimate, MobilitySubsystem::MobilityMode::ACKERMANN, steerAngle, reverse, curr));                
        }
    }

    // Crab
    for (double steerAngle : {5.0, -5.0, 10.0, -10.0, 15.0, -15.0, 20.0, -20.0, 25.0, -25.0}) {
        for (int reverse = 0; reverse <= 1; reverse++) {
            double step = reverse ? -PathingConstants::stepSize : PathingConstants::stepSize;
            
            double x = curr->x + (step * std::cos(curr->angle + (steerAngle / 180.0 * M_PI)));
            double y = curr->y + (step * std::sin(curr->angle + (steerAngle / 180.0 * M_PI)));
            double a = curr->angle;

            double cost = curr->cost + calculateSteerCost(curr->motionType, curr->motionData, MobilitySubsystem::MobilityMode::CRAB, steerAngle);
            double angleToTarget = std::abs(std::atan2(targetPos.y - y, targetPos.x - x) - a);
            double estimate = ((curr->dist(targetPos) * 0.3) + angleToTarget) * 0.4;

            successors.push_back(new Node(x, y, a, cost, estimate, MobilitySubsystem::MobilityMode::CRAB, steerAngle, reverse, curr));                
        }
    }

    // Zero Point Turn
    for (double turnAmount : {15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 105.0, 120.0, 135.0, 150.0, 165.0, 180.0}) {
        for (int reverse = 0; reverse <= 1; reverse++) {
            double x = curr->x;
            double y = curr->y;
            double a = curr->angle + (turnAmount / 180.0 * M_PI);

            double cost = curr->cost + calculateSteerCost(curr->motionType, curr->motionData, MobilitySubsystem::MobilityMode::ZERO_POINT, turnAmount);
            double angleToTarget = std::abs(std::atan2(targetPos.y - y, targetPos.x - x) - a);
            double estimate = ((curr->dist(targetPos) * 0.3) + angleToTarget) * 0.4;

            successors.push_back(new Node(x, y, a, cost, estimate, MobilitySubsystem::MobilityMode::ZERO_POINT, turnAmount, reverse, curr));                
        }
    }

    return successors;
}

void Pathfinding::addObject(Coord pos, double radius) {
    objects.push_back(std::make_tuple(pos, radius, 0));
}

Coord Pathfinding::getTargetPos() {
    return targetPos;
} 

double Pathfinding::getTargetAngle() {
    return targetAngle;
}