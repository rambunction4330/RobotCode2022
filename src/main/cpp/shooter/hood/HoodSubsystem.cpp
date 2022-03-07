// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "shooter/hood/HoodSubsystem.h"

HoodSubsystem::HoodSubsystem() = default;

// This method will be called once per scheduler run
void HoodSubsystem::Periodic() {}

void HoodSubsystem::setPosition(units::angle::radian_t position) {
    positionController.setPosition(position);
}

units::angle::radian_t HoodSubsystem::getPosition() {
    return positionController.getPosition();
}
