// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>

#include "shooter/ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem(const units::meter_t wd) : wheelDiameter(wd) {}

void ShooterSubsystem::spinTo(units::angular_velocity::radians_per_second_t vel) {
  leftMainShooterMotor.setVelocity(vel);
}

void ShooterSubsystem::spinTo(units::velocity::meters_per_second_t vel) {
  leftMainShooterMotor.setVelocity(
    units::angular_velocity::radians_per_second_t(vel * (1_rad / (wheelDiameter / 2)))
  );
}

void ShooterSubsystem::stop() {
  leftMainShooterMotor.setVelocity(0.0_tps);
}

units::angular_velocity::radians_per_second_t ShooterSubsystem::getAngularVelocity() {
  return leftMainShooterMotor.getVelocity();
}
units::velocity::meters_per_second_t ShooterSubsystem::getLinearVelocity() {
  return leftMainShooterMotor.getVelocity() / 1_rad * wheelDiameter;
}


// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {}


