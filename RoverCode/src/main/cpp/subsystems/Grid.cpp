#pragma once

#include "subsystems/Grid.h"
#include <frc/smartdashboard/SmartDashboard.h>

GridSubsystem::GridSubsystem() {
    //initialize grid as 0,0
}

//TODO find origin
GridSubsystem::locateOrigin() {
    //if(!Vision::isTagVisible(0) != !Vision::isTagVisible(1))
        //TODO spin robot
    //if tags visible, then find location of rover in grid

    //Get distances and angles from the camera
    double d0 = Vision::getTagDistance(0);
    double a0 = Vision::getTagAngle(0);
    double n0 = Vision::getTagNormal(0);
    double d1 = Vision::getTagDistance(1);
    double a1 = Vision::getTagAngle(1);
    double n1 = Vision::getTagNormal(1);

    //Convert polar coordinates to cartesian (relative to each tag)
//     double x0 = d0 * cos(a0);
//     double y0 = d0 * sin(a0);
//     double x1 = d1 * cos(a1);
//     double y1 = d1 * sin(a1);
}

GridSubsystem::updateRovPos(double dx, double dy) { //delta x and delta y
    double tx = rovX+dx;
    double ty = rovY+dy;

    //checks rovers position and says if we are outside the arena
    if (tx < GridConstants::arenaWidth && tx > 0 && 
        ty < GridConstants::arenaHeight && ty > 0){
            rovX=tx;
            rovY=ty;
        }
}

int GridSubsystem::getCell(int x, int y) {
    return grid[y][x];
}

void GridSubsystem::setCell(int x, int y, int val) {
    grid[y][x] = val;
}