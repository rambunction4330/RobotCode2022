// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>

#include "shooter/ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem() {}

void ShooterSubsystem::spinTo(units::velocity::meters_per_second_t vel) {
  flywheel.setVelocity(vel);
}

void ShooterSubsystem::stop() {
  flywheel.setVelocity(0.0_mps);
}

units::velocity::meters_per_second_t ShooterSubsystem::getVelocity() {
  return flywheel.getVelocity();
}


// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {}


