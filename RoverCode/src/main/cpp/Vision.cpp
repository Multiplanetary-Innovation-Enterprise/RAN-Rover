#pragma once

#include <Vision.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/shuffleboard/Shuffleboard.h"
#include "cameraserver/CameraServer.h"

Vision::Vision() {
    
    nt::NetworkTableInstance tableInstance = nt::NetworkTableInstance::GetDefault();
    std::string streamNames[1];
    streamNames[0] = "mjpeg:http://192.168.1.127:6463/front";
    tableInstance.GetEntry("/CameraPublisher/FrontColor/streams").SetStringArray(streamNames);
}