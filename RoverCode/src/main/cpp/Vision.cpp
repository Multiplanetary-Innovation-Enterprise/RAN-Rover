#pragma once

#include <Vision.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/shuffleboard/Shuffleboard.h"
#include "cameraserver/CameraServer.h"

Vision::Vision() {

    // WIP: Attempting to link camera stream address to network tables so camera server can pull stream
    nt::NetworkTableInstance tableInstance = nt::NetworkTableInstance::GetDefault();
    std::string streamNames[1];
    streamNames[0] = "mjpeg:http://192.168.1.127:6463/front";
    tableInstance.GetEntry("/CameraPublisher/FrontColor/streams").SetStringArray(streamNames);

    tagTable = netTable.GetTable("Vision/Beacons");
}

void Vision::IdentifyTags() {
    for (std::string tag : tagTable->GetSubTables()) {
        
        int tagId = atoi(tag.substr(4).c_str());
        if (tagAngles.find(tagId) != tagAngles.end())
            continue;

        nt::DoubleSubscriber tagAngle = tagTable->GetDoubleTopic(tag + "/Angle").Subscribe(0);
        nt::DoubleSubscriber tagDistance = tagTable->GetDoubleTopic(tag + "/Distance").Subscribe(0);
        tagAngles[tagId] = &tagAngle;
        tagDistances[tagId] = &tagDistance;
    }
}

bool Vision::isTagVisible(int tagId) {
    // Need to establish a standard communicated from the Pi to determine if tag is visible
    if (tagAngles.find(tagId) != tagAngles.end())
        return true;
    return false;
}

double Vision::getTagAngle(int tagId) {
    if (tagAngles.find(tagId) != tagAngles.end())
        return tagAngles[tagId]->Get();
    wpi::outs() << "Tag " << std::to_string(tagId) << " Angle Not Found\n";
    return 0;
} 

double Vision::getTagDistance(int tagId) {
    if (tagDistances.find(tagId) != tagDistances.end())
        return tagDistances[tagId]->Get();
    wpi::outs() << "Tag " << std::to_string(tagId) << " Distance Not Found\n";
    return 0;
}