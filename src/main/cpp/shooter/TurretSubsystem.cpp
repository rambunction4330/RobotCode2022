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

void TurretSubsystem::spinOffset(units::angle::radian_t offset) {
  positionController.spinOffset(offset);
}

void TurretSubsystem::spinOffset(units::length::meter_t offset) {
  positionController.spinOffset(offset * (1_rad / (wheelDiameter / 2)));
}

units::length::meter_t TurretSubsystem::getLinearPosition() {
  return positionController.getPosition() * ((wheelDiameter / 2) / 1_rad);
}

units::angle::radian_t TurretSubsystem::getAngularPosition() {
  return positionController.getPosition();
}

bool TurretSubsystem::isAtPosition(units::angle::radian_t pos) {
  return positionController.atPosition(pos);
}

bool TurretSubsystem::isAtPosition(units::length::meter_t pos) {
  return positionController.atPosition(pos * (1_rad / (wheelDiameter / 2)));
}