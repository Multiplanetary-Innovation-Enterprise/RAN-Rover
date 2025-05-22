#pragma once
#define _USE_MATH_DEFINES

#include "Constants.h"
#include "Vision.h"
#include "Localization.h"
#include <wpi/raw_ostream.h>
#include <vector>
#include <queue>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include "sendables/Coord.h"
#include <frc/Timer.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "subsystems/Mobility.h"
#include <future>

struct Node : Coord {
    double angle; // Facing direction in radians
    double cost; // Cost from Start
    double estimate; // Estimate to Target
    MobilitySubsystem::MobilityMode motionType;
    double motionData;
    bool reverse;
    Node* parent;

    Node(Coord pos, double angle, double cost, double estimate, MobilitySubsystem::MobilityMode motionType, double motionData, bool reverse, Node* parent = nullptr)
        : angle(angle), cost(cost), estimate(estimate), motionType(motionType), motionData(motionData), reverse(reverse), parent(parent) {
        this->x = pos.x;
        this->y = pos.y;
    }

    Node(double x, double y, double angle, double cost, double estimate, MobilitySubsystem::MobilityMode motionType, double motionData, bool reverse, Node* parent = nullptr)
        : angle(angle), cost(cost), estimate(estimate), motionType(motionType), motionData(motionData), reverse(reverse), parent(parent) {
        this->x = x;
        this->y = y;
    }

    double totalCost() const { return cost + estimate; }
};

struct CompareNode {
    bool operator()(Node* a, Node* b) {
        return a->totalCost() > b->totalCost();
    }
};

class Pathfinding {
    public:
        Pathfinding(Vision* vision, Localization* local);
        void Init();
        void Periodic();

        Node* queryPath();
        bool isValid(Node* node); // If node is not colliding with the wall or obstacle.
        bool reachedTarget(Node* node); // If node is within a tolerance of the target position.

        void setTarget(Coord position, double angle);
        Coord getTargetPos();
        double getTargetAngle();
    private:
        Vision* vision;
        Localization* local;
        Coord targetPos;
        double targetAngle;
        frc::Timer recalcTimer;
        frc::Timer readObjectsTimer;
        units::time::second_t recalcPeriod{5.0};
        units::time::second_t readObjectsPeriod{2.0};
        bool hasQueried;

        std::future<Node*> path;
        Node* findPath(Coord startPos, double startAngle);
        
        std::string makeVisitedKey(Node* node); // Used for consistent formatting of visitedSet keys
        double calculateSteerCost(MobilitySubsystem::MobilityMode currMode, double currSteer, MobilitySubsystem::MobilityMode nextMode, double nextSteer); // Determines the time it takes to configure steering to new position.

        std::vector<Node*> generateSuccessors(Node* curr);

        void addObject(Coord pos, double radius);

        std::vector<std::tuple<Coord, double, int>> objects; // Stored as vector of (Position, Radius, Collision Count)
};