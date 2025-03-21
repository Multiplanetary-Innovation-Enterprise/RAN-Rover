#pragma once

#include "Constants.h"
#include "Vision.h"

class GridSubsystem : public {
    public:
        GridSubsystem();
        void locateOrigin();

        int getCell(double x, double y);
        void setCell(double x, double y, int val);
        std::array<double,2> getRoverPos();

        void updateRovPos(double dx, double dy);
    private:    
        std::array<std::array<int,w>,h> grid;
        std::array<double, 2> rovPos(0,0);
}