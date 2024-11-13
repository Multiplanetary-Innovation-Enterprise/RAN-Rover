#include <AHRS.h>
#pragma once

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableRegistry.h>

class IMUSendable : public AHRS {
    public:
    IMUSendable(frc::SPI::Port port);
    void InitSendable(wpi::SendableBuilder& builder) override;
};