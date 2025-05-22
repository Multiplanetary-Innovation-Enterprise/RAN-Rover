#include <Localization.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include <math.h>
#include <cmath>

Localization::Localization(Vision* v, IMUSendable* imu) : vision(v), imu(imu) {
    const std::string arenaKSC = "KSC";
    const std::string arenaUCF = "UCF";
    arenaChooser.SetDefaultOption(arenaKSC, Arena::KSC);
    arenaChooser.AddOption(arenaUCF, Arena::UCF);

    frc::SmartDashboard::PutBoolean("Localization/Use IMU Positioning", false);
    frc::SmartDashboard::PutData("Localization/Arena", &arenaChooser);
    
    frc::SmartDashboard::PutNumber("Localization/Offsets/Front X", frontOffset.x);
    frc::SmartDashboard::SetPersistent("Localization/Offsets/Front X");
    frc::SmartDashboard::PutNumber("Localization/Offsets/Front Y", frontOffset.y);
    frc::SmartDashboard::SetPersistent("Localization/Offsets/Front Y");

    frc::SmartDashboard::PutNumber("Localization/Offsets/Back X", backOffset.x);
    frc::SmartDashboard::SetPersistent("Localization/Offsets/Back X");
    frc::SmartDashboard::PutNumber("Localization/Offsets/Back Y", backOffset.y);
    frc::SmartDashboard::SetPersistent("Localization/Offsets/Back Y");
}

void Localization::Init() {
    arena = arenaChooser.GetSelected();
}

void Localization::Periodic() {

    vision->IdentifyTags();
    Vision::Camera tag0Visible = vision->isTagVisible(0);
    Vision::Camera tag1Visible = vision->isTagVisible(1);

    if (tag0Visible || tag1Visible) {

        deltaTimer.Stop();
        deltaTimer.Reset();
        // Allow calibration immediately if sees tags, but when in continuous view, only so often.
        if (calibrateTimer.HasElapsed(units::time::second_t{0.01}) && !calibrateTimer.AdvanceIfElapsed(units::time::second_t{2.0}))
            return;

        calibrateTimer.Start();

        Coord camPos{0.0, 0.0};
        Coord camOffset{0.0, 0.0};
        Coord tag0Pos = vision->getTagPos(0);
        Coord tag1Pos = vision->getTagPos(1);
        double camAngle = 0.0;
    
        if (vision->isTagVisible(0) && vision->isTagVisible(1)) { 
            camPos.x = fabs(vision->getTagDistance(0) * cos((vision->getTagNormal(0) - vision->getTagAngle(0)) / 180.0 * M_PI));
            camPos.y = fabs(vision->getTagDistance(1) * cos((vision->getTagNormal(1) + vision->getTagAngle(1)) / 180.0 * M_PI));

            tag0Pos.y = fabs(camPos.y - vision->getTagDistance(0) * sin((vision->getTagNormal(0) - vision->getTagAngle(0)) / 180.0 * M_PI));
            tag1Pos.x = fabs(camPos.x - vision->getTagDistance(1) * sin((vision->getTagNormal(1) + vision->getTagAngle(1)) / 180.0 * M_PI));

            vision->setTagPos(0, tag0Pos);
            vision->setTagPos(1, tag1Pos);
        } else if (vision->isTagVisible(0) && tag0Pos.y != 0) {
            camPos.x = fabs(vision->getTagDistance(0) * cos((vision->getTagNormal(0) - vision->getTagAngle(0)) / 180.0 * M_PI));
            camPos.y = fabs(vision->getTagDistance(0) * sin((vision->getTagNormal(0) - vision->getTagAngle(0)) / 180.0 * M_PI) + tag0Pos.y);
        } else if (vision->isTagVisible(1) && tag1Pos.x != 0) {
            camPos.x = fabs(vision->getTagDistance(1) * sin((vision->getTagNormal(1) + vision->getTagAngle(1)) / 180.0 * M_PI) + tag1Pos.x);
            camPos.y = fabs(vision->getTagDistance(1) * cos((vision->getTagNormal(1) + vision->getTagAngle(1)) / 180.0 * M_PI));
        } else {
            return;
        }

        if (tag0Visible == Vision::FRONT) {
            frontOffset.x = frc::SmartDashboard::GetNumber("Localization/Offsets/Front X", frontOffset.x);
            frontOffset.y = frc::SmartDashboard::GetNumber("Localization/Offsets/Front Y", frontOffset.y);
            camOffset == frontOffset;
            camAngle = (90.0 - (vision->getTagNormal(0) - vision->getTagAngle(0))) / 180.0 * M_PI;
        } else if (tag0Visible == Vision::BACK) {
            backOffset.x = frc::SmartDashboard::GetNumber("Localization/Offsets/Back X", backOffset.x);
            backOffset.y = frc::SmartDashboard::GetNumber("Localization/Offsets/Back Y", backOffset.y);
            camOffset == backOffset;
            camAngle = (90.0 - (vision->getTagNormal(0) - vision->getTagAngle(0))) / 180.0 * M_PI;
        } else if (tag1Visible == Vision::FRONT) {
            frontOffset.x = frc::SmartDashboard::GetNumber("Localization/Offsets/Front X", frontOffset.x);
            frontOffset.y = frc::SmartDashboard::GetNumber("Localization/Offsets/Front Y", frontOffset.y);
            camOffset == frontOffset;
            camAngle = (vision->getTagNormal(1) + vision->getTagAngle(1)) / 180.0 * M_PI;
        } else if (tag1Visible == Vision::BACK) {
            backOffset.x = frc::SmartDashboard::GetNumber("Localization/Offsets/Back X", backOffset.x);
            backOffset.y = frc::SmartDashboard::GetNumber("Localization/Offsets/Back Y", backOffset.y);
            camOffset == backOffset;
            camAngle = (vision->getTagNormal(1) + vision->getTagAngle(1)) / 180.0 * M_PI;
        }

        Coord angleFactor{sin(camAngle), cos(camAngle)};
        camOffset *= angleFactor;
        roverCenter = camPos + camOffset;
        yawZeroOffset = -camAngle * 180.0 / M_PI;
        imu->ZeroYaw();
        if (!isnan(roverCenter.x)) frc::SmartDashboard::PutNumber("Localization/Rover X", roverCenter.x);
        if (!isnan(roverCenter.y)) frc::SmartDashboard::PutNumber("Localization/Rover Y", roverCenter.y);
        if (!isnan(camPos.x)) frc::SmartDashboard::PutNumber("Localization/Camera X", camPos.x);
        if (!isnan(camPos.y)) frc::SmartDashboard::PutNumber("Localization/Camera Y", camPos.y);
        if (!isnan(camOffset.y)) frc::SmartDashboard::PutNumber("Localization/Camera Offset X", camOffset.x);
        if (!isnan(camOffset.y)) frc::SmartDashboard::PutNumber("Localization/Camera Offset Y", camOffset.y);
        if (!isnan(camAngle)) frc::SmartDashboard::PutNumber("Localization/Rover Angle", yawZeroOffset);
        if (!isnan(tag0Pos.y)) frc::SmartDashboard::PutNumber("Localization/Tag 0 Y", tag0Pos.y);
        if (!isnan(tag1Pos.x)) frc::SmartDashboard::PutNumber("Localization/Tag 1 X", tag1Pos.x);
        return;
    } else if (frc::SmartDashboard::GetBoolean("Localization/Use IMU Positioning", false)) {
        calibrateTimer.Stop();
        calibrateTimer.Reset();

        Coord delta{0.0, 0.0};
        double yaw = getRoverAngle();
        // May need to use Acceleration as the Velocity functions state they may be inaccurate.
        // Also may need to triangulate based on if the Z axis is absorbing some movement. This is expected as the IMU is pitched down.
        double imuX = imu->GetVelocityZ(); // From meters per second to meters per 20 milliseconds
        double imuY = imu->GetVelocityY(); // From meters per second to meters per 20 milliseconds
        double movementYaw = atan(imuX / imuY);
        double movementMag = sqrt(pow(imuX, 2) + pow(imuY, 2));
        delta.x = sin(movementYaw + (yaw / 180.0 / M_PI)) * movementMag / deltaTimer.Get().value();
        delta.y = cos(movementYaw + (yaw / 180.0 / M_PI)) * movementMag / deltaTimer.Get().value();
        updateRoverCenter(delta);
        deltaTimer.Start();
        deltaTimer.Reset();
        if (!isnan(roverCenter.x)) frc::SmartDashboard::PutNumber("Localization/Rover X", roverCenter.x);
        if (!isnan(roverCenter.y)) frc::SmartDashboard::PutNumber("Localization/Rover Y", roverCenter.y);
        if (!isnan(yaw)) frc::SmartDashboard::PutNumber("Localization/Rover Angle", yaw);
    } else {
        calibrateTimer.Stop();
        calibrateTimer.Reset();
    }
}

void Localization::setRoverCenter(Coord center) {
    roverCenter = center;
    wpi::outs() << "New Rover Position Set: " << center.toStr() << "\n";
}

void Localization::updateRoverCenter(Coord delta) {
    Coord newRoverCenter = roverCenter + delta;
    if (newRoverCenter > (PathingConstants::roverSize * Coord{0.5, 0.5}) && newRoverCenter < (getArenaSize() + (PathingConstants::roverSize * Coord{-0.5, -0.5})))
        roverCenter = newRoverCenter;
    //wpi::outs() << "Rover Position Trying to Leave Arena Bounds: " << newRoverCenter.toStr() << "\n";
}

Coord Localization::getRoverCenter() {
    return roverCenter;
}

double Localization::getRoverAngle() {
    return imu->GetYaw() + yawZeroOffset;
}

///////////////////////////////
// Obtaining Arena Constants //
///////////////////////////////

Coord Localization::getArenaSize() {
    if (arena == Arena::KSC)
        return KSC_ArenaConstants::size; 
    else
        return UCF_ArenaConstants::size;
}

std::array<Coord, 2> Localization::tBounds() { // Traversal Zone Bounds
    if (arena == Arena::KSC)
        return {KSC_ArenaConstants::traversalZonePos, KSC_ArenaConstants::traversalZonePos + KSC_ArenaConstants::traversalZoneSize};
    else
        return {UCF_ArenaConstants::traversalZonePos, UCF_ArenaConstants::traversalZonePos + UCF_ArenaConstants::traversalZoneSize};
}

std::array<Coord, 2> Localization::eBounds() { // Excavation Zone Bounds
    if (arena == Arena::KSC)
        return {KSC_ArenaConstants::excavationZonePos, KSC_ArenaConstants::excavationZonePos + KSC_ArenaConstants::excavationZoneSize};
    else
        return {UCF_ArenaConstants::excavationZonePos, UCF_ArenaConstants::excavationZonePos + UCF_ArenaConstants::excavationZoneSize};
}

std::array<Coord, 2> Localization::cBounds() { // Construction Zone Bounds
    if (arena == Arena::KSC)
        return {KSC_ArenaConstants::constructionZonePos, KSC_ArenaConstants::constructionZonePos + KSC_ArenaConstants::constructionZoneSize};
    else
        return {UCF_ArenaConstants::constructionZonePos, UCF_ArenaConstants::constructionZonePos + UCF_ArenaConstants::constructionZoneSize};
}

Coord Localization::bermCenter() { // Berm Bounds
    if (arena == Arena::KSC)
        return KSC_ArenaConstants::bermCenter;
    else
        return UCF_ArenaConstants::bermCenter;
}

std::tuple<Coord, double> Localization::getDepositionTarget() { // Where is the position we need to be in to deposit for each arena.
    const double offset = PathingConstants::roverSize.y + 0.1;
    if (arena == Arena::KSC)
        return std::make_tuple(KSC_ArenaConstants::bermCenter + Coord{0, offset}, 0.0);
    else {
        return std::make_tuple(UCF_ArenaConstants::bermCenter - Coord{offset, 0}, -90.0);
    }
}