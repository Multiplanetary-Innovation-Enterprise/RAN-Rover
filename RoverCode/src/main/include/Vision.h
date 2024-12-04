#pragma once

#include <wpi/raw_ostream.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleTopic.h>
#include <map>
#include "Constants.h"

class Vision {
    public:        
        Vision();

        void IdentifyTags();

        bool isTagVisible(int id);
        double getTagDistance(int id);
        double getTagAngle(int id);
        double getTagNormalAngle(int id);
    private:

        nt::NetworkTableInstance netTable{nt::NetworkTableInstance::GetDefault()};
        std::shared_ptr<nt::NetworkTable> tagTable;

        std::map<int, nt::DoubleSubscriber*> tagAngles;
        std::map<int, nt::DoubleSubscriber*> tagDistances;
};