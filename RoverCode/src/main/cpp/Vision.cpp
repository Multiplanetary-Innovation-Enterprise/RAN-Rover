#pragma once

#include <Vision.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/shuffleboard/Shuffleboard.h"
#include "cameraserver/CameraServer.h"

Vision::Vision() {

    // WIP: Attempting to link camera stream address to network tables so camera server can pull stream
    std::string streamNames[1];
    streamNames[0] = "mjpeg:http://192.168.1.127:6463/front";
    netTable.GetEntry("/CameraPublisher/FrontColor/streams").SetStringArray(streamNames);
    tagTable = netTable.GetTable("Vision/Beacons");
}

void Vision::IdentifyTags() {

    for (std::string tag : tagTable->GetSubTables()) {

        int tagId = atoi(tag.substr(4).c_str());
        nt::BooleanSubscriber tagVisible = tagTable->GetBooleanTopic(tag + "/Visible").Subscribe(false);
        nt::DoubleSubscriber tagAngle = tagTable->GetDoubleTopic(tag + "/Angle").Subscribe(0);
        nt::DoubleSubscriber tagNormal = tagTable->GetDoubleTopic(tag + "/Normal").Subscribe(0);
        nt::DoubleSubscriber tagDistance = tagTable->GetDoubleTopic(tag + "/Distance").Subscribe(0);

        bool tagVisibility = tagVisible.Get();
        if ((tagLastSeen == -1 || !tagVisibilities[tagId]) && tagVisibility)
            tagLastSeen = tagId;

        tagVisibilities[tagId] = tagVisibility;
        tagAngles[tagId] = tagAngle.Get();
        tagNormals[tagId] = tagNormal.Get();
        tagDistances[tagId] = tagDistance.Get();
    }
}

std::vector<int> Vision::getVisibleTags() {
    std::vector<int> visibleTags;

    for (int i = 0; i < 4; i++) {
        if (isTagVisible(i)) 
            visibleTags.push_back(i);
    }

    return visibleTags;
}

int Vision::getTagLastSeen() {
    return tagLastSeen;
}

bool Vision::isTagVisible(int tagId) {
    if (tagId < 0 || tagId > 3)
        return false;
    // Need to establish a standard communicated from the Pi to determine if tag is visible
    if (tagVisibilities.find(tagId) != tagVisibilities.end()) {
        bool visible = tagVisibilities[tagId];
        // wpi::outs() << "Tag " << std::to_string(tagId) << " Visibility " << (visible ? "True" : "False") << "\n";
        return visible;
    }
    wpi::outs() << "Tag " << std::to_string(tagId) << " Visibility Not Found\n";
    return false;
}

double Vision::getTagAngle(int tagId) {
    if (tagAngles.find(tagId) != tagAngles.end()) {
        double angle = tagAngles[tagId];
        // wpi::outs() << "Tag " << std::to_string(tagId) << " Angle is " << std::to_string(angle) << "\n";
        return angle;
    }
    wpi::outs() << "Tag " << std::to_string(tagId) << " Angle Not Found\n";
    return 180; // If tag is not visible, is likely somewhere behind us
} 

double Vision::getTagNormal(int tagId) {
    if (tagNormals.find(tagId) != tagNormals.end()) {
        double normal = tagNormals[tagId];
        // wpi::outs() << "Tag " << std::to_string(tagId) << " Normal is " << std::to_string(normal) << "\n";
        return normal;
    }
    wpi::outs() << "Tag " << std::to_string(tagId) << " Normal Not Found\n";
    return 0;
}

double Vision::getTagDistance(int tagId) {
    if (tagDistances.find(tagId) != tagDistances.end()) {
        double dist = tagDistances[tagId];
        // wpi::outs() << "Tag " << std::to_string(tagId) << " Distance is " << std::to_string(dist) << "\n";
        return dist;
    }
    wpi::outs() << "Tag " << std::to_string(tagId) << " Distance Not Found\n";
    return 0;
}