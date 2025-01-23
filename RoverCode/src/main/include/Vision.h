#pragma once

#include <wpi/raw_ostream.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleTopic.h>
#include <networktables/BooleanTopic.h>
#include <map>
#include "Constants.h"

class Vision {
    public:        
        Vision();

        void IdentifyTags();

        std::vector<int> getVisibleTags();
        int getTagLastSeen();

        bool isTagVisible(int id);
        double getTagAngle(int id);
        double getTagNormal(int id);
        double getTagDistance(int id);
    private:

        nt::NetworkTableInstance netTable{nt::NetworkTableInstance::GetDefault()};
        std::shared_ptr<nt::NetworkTable> tagTable;

        int tagLastSeen = -1;
        std::map<int, bool> tagVisibilities;
        std::map<int, double> tagAngles;
        std::map<int, double> tagNormals;
        std::map<int, double> tagDistances;
};