// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "climber/ClimberSubsystem.h"

ClimberSubsystem::ClimberSubsystem() = default;

void ClimberSubsystem::climb() {
  talon.Set(-1.0);
}

void ClimberSubsystem::lower() {
  talon.Set(0.5);
}

void ClimberSubsystem::stop() {
  talon.Set(0.0);
}

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}
