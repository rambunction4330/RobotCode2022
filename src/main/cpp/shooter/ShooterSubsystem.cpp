// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>

#include "shooter/ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem()  : wheelDiameter(1_m) {}
ShooterSubsystem::ShooterSubsystem(const units::meter_t wd) : wheelDiameter(wd) {}

void ShooterSubsystem::spinTo(units::unit_t<units::compound_unit<units::angle::radian, units::inverse<units::time::second>>> vel) {
  leftMainShooterMotor.setVelocity(vel);
}

void ShooterSubsystem::spinTo(units::unit_t<units::compound_unit<units::length::meter, units::inverse<units::time::second>>> vel) {
  leftMainShooterMotor.setVelocity(
    units::angular_velocity::radians_per_second_t(vel * (1_rad / (wheelDiameter / 2)))
  );
}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {}


