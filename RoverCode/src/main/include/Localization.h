#pragma once
#define _USE_MATH_DEFINES

#include "Constants.h"
#include "Vision.h"
#include "sendables/IMUSendable.h"
#include <wpi/raw_ostream.h>
#include <vector>
#include <queue>
#include <string>
#include <unordered_map>
#include "sendables/Coord.h"
#include <frc/Timer.h>
#include <frc/smartdashboard/SendableChooser.h>

struct Cell : Coord {
    bool occupied = false;
    int costFromStart; // Formally g
    int estimateCostToTarget; // Formally h
    Cell* parent;
    
    Cell(int x, int y, int costFromStart, int estimateCostToTarget, Cell* parent = nullptr):  costFromStart(costFromStart), estimateCostToTarget(estimateCostToTarget), parent(parent) {
        this->x = x;
        this->y = y;
    }
    
    int costOfPath() const { return costFromStart + estimateCostToTarget; } // Formally f
};

class Localization {
    public:

        enum Arena {
            KSC,
            UCF
        };

        Localization(Vision* vision, IMUSendable* imu);
        void Init();
        void Periodic();

        std::vector<Coord> findPath(Coord target);

        void clearGrid();
        void addObstacle(Coord pos, double radius);
        std::string displayGrid(Coord target, std::vector<Coord> path = {});
        
        void setRoverCenter(Coord pos);
        void updateRoverCenter(Coord delta);
        Coord getRoverCenter();
        double getRoverAngle();
    private:
        frc::SendableChooser<Arena> arenaChooser;
        Arena arena = KSC;
        Vision* vision;
        IMUSendable* imu;
        frc::Timer calibrateTimer;
        frc::Timer deltaTimer;

        // Offset required to reach center point of rover
        Coord frontOffset{0.2032, 0.6096}; // 8 inches by 24 inches
        Coord backOffset{0, -0.75};
        Coord roverSize{0.75, 1.5};
        double cellSize = 0.1;
        double yawZeroOffset = 0.0;

        // All measurements throughout this class will be in meters.
        std::vector<std::vector<Cell>> grid;
        Coord roverCenter{1.0, 1.0};

        bool isValid(Coord c);
        Coord getRoverSize();
        double getRoverDiagSize();
        double getCellSize();
        Coord getArenaSize();
        std::array<Coord, 2> tBounds(); // Traversal Zone Bounds
        std::array<Coord, 2> eBounds(); // Excavation Zone Bounds
        std::array<Coord, 2> cBounds(); // Construction Zone Bounds
        Coord bermCenter();
};