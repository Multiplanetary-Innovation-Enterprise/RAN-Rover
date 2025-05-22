#pragma once

#include <wpi/raw_ostream.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/BooleanTopic.h>
#include <map>
#include <queue>
#include <vector>
#include "sendables/Coord.h"
#include "Constants.h"

struct Object {
    double dist, minAngle, maxAngle;

    Object(double dist, double minAngle, double maxAngle) : dist(dist), minAngle(minAngle), maxAngle(maxAngle) {}

    std::string toStr() const {
        return std::to_string(dist) + "," + std::to_string(minAngle) + "~" + std::to_string(maxAngle) + ">";
    }
};

class Vision {
    public:      

        enum Camera {
            NONE = 0,
            FRONT = 1,
            BACK = 2,
        };

        Vision();

        std::vector<Object> IdentifyObjects();
        void IdentifyTags();

        std::vector<int> getVisibleTags();
        int getTagLastSeen();

        Camera isTagVisible(int id);
        double getTagAngle(int id);
        double getTagNormal(int id);
        double getTagDistance(int id);
        Coord getTagPos(int id);
        void setTagPos(int id, Coord);
    private:

        nt::NetworkTableInstance netTable{nt::NetworkTableInstance::GetDefault()};
        std::shared_ptr<nt::NetworkTable> table;

        int tagLastSeen = -1;
        std::map<int, bool> tagVisibilities;
        std::map<int, std::string> tagSeenBy;
        std::map<int, double> tagAngles;
        std::map<int, double> tagNormals;
        std::map<int, double> tagDistances;
        std::map<int, Coord*> tagPos;
};