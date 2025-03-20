#pragma once

#include "subsystems/Grid.h"
#include <frc/smartdashboard/SmartDashboard.h>

GridSubsystem::GridSubsystem() {
    //initialize grid
    for (int y=0; y<GridConstants::width;y++) {
        for (int x=0; x<GridConstants::height;x++) {
            grid[y][x]=0;
        }
    }
}

//TODO find origin
GridSubsystem::locateOrigin() {
    //TODO spin robot
    //if tags visible, then find location of rover in grid

    //Get distances and angles from the camera
    double d0 = Vision::getTagDistance(0);
    double a0 = Vision::getTagAngle(0);
    double d1 = Vision::getTagDistance(1);
    double a1 = Vision::getTagAngle(1);

    //Convert polar coordinates to cartesian (relative to each tag)
    double x0 = d0 * cos(a0);
    double y0 = d0 * sin(a0);
    double x1 = d1 * cos(a1);
    double y1 = d1 * sin(a1);
}