// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "shooter/TurretSubsystem.h"

TurretSubsystem::TurretSubsystem(units::length::meter_t wd) : wheelDiameter(wd) {}

// This method will be called once per scheduler run
void TurretSubsystem::Periodic() {}

void TurretSubsystem::spinTo(units::length::meter_t pos) {
  positionController.setPosition(pos * (1_rad / (wheelDiameter / 2)));
}

void TurretSubsystem::spinTo(units::angle::radian_t pos) {
  positionController.setPosition(pos);
}

units::length::meter_t TurretSubsystem::getLinearPosition() {
  return positionController.getPosition() * ((wheelDiameter / 2) / 1_rad);
}

units::angle::radian_t TurretSubsystem::getAngularPosition() {
  return positionController.getPosition();
}