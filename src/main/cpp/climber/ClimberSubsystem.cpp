// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "climber/ClimberSubsystem.h"

ClimberSubsystem::ClimberSubsystem() = default;

void ClimberSubsystem::extendArm()  {
    armTalon.Set(1.0);
}

void ClimberSubsystem::retractArm() {
    armTalon.Set(-1.0);
}

void ClimberSubsystem::stopArm() {
  armTalon.Set(0.0);
}

void ClimberSubsystem::winchTighten() {
    winchTalon.Set(1.0);
}

void ClimberSubsystem::winchRelease() {
    winchTalon.Set(-1.0);
}

void ClimberSubsystem::winchStop() {
    winchTalon.Set(0.0);
}

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}
