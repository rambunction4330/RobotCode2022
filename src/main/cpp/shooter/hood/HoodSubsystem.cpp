// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "shooter/hood/HoodSubsystem.h"
#include "rmb/io/log.h"

HoodSubsystem::HoodSubsystem() {
    positionController.resetRefrence(15.0_deg);
    positionController.setMinPosition(21.0_deg);
    positionController.setMaxPosition(42.0_deg);
}

// This method will be called once per scheduler run
void HoodSubsystem::Periodic() {
    //wpi::outs() << "hood position: " << (units::degree_t) positionController.getPosition() << wpi::endl;
}

void HoodSubsystem::setPosition(units::angle::radian_t position) {
    positionController.setPosition(position);
}

units::angle::radian_t HoodSubsystem::getPosition() {
    return positionController.getPosition();
}
