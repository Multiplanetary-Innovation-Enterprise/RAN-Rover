#pragma once

#include <wpi/raw_ostream.h>
#include "Constants.h"

class Vision {
    public:        
        Vision();

        bool isTagVisible(int id);
        double getTagDistance(int id);
        double getTagDirection(int id);
        double getTagNormalAngle(int id);
};