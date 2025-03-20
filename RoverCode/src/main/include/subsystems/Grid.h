#pragma once

#include "Constants.h"
#include "Vision.h"

class GridSubsystem : public {
    public:
        GridSubsystem();
        void locateOrigin();
        
        void currentCell();

        void getCell(x, y);
        void setCell(x, y, int);
        void getRoverCell();

        void int getRows();
        void int getCols();
    private:    
        std::array<std::array<int,w>,h> grid;
        int currentRow, currentCol;
        int rows, cols;

        bool isValid(int row, int col);
}