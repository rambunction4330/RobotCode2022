// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "climber/ClimberSubsystem.h"

ClimberSubsystem::ClimberSubsystem() = default;

void ClimberSubsystem::extendArm()  {
  leftTalon.Set(0.5);
  rightTalon.Set(-0.5);
}

void ClimberSubsystem::retractArm() {
  leftTalon.Set(-1.0);
  rightTalon.Set(1.0);
}

void ClimberSubsystem::stopArm() {
  leftTalon.Set(0.0);
  rightTalon.Set(0.0);  
}

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}
