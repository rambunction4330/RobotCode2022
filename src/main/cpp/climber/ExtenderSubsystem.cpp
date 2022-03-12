// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "climber/ExtenderSubsystem.h"

ExtenderSubsystem::ExtenderSubsystem() = default;

void ExtenderSubsystem::extendArm()  {
  talon.Set(-0.5);
}

void ExtenderSubsystem::retractArm() {
  talon.Set(0.5);
}

void ExtenderSubsystem::stopArm() {
  talon.Set(0.0); 
}

// This method will be called once per scheduler run
void ExtenderSubsystem::Periodic() {}