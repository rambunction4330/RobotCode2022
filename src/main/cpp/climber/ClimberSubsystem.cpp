// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "climber/ClimberSubsystem.h"

ClimberSubsystem::ClimberSubsystem() = default;

void ClimberSubsystem::ExtendArm()  {

    leftTalon.Set(5.0);
    rightTalon.Set(5.0);
    ClimberState = true;
}

void ClimberSubsystem::RetractArm() {

    leftTalon.Set(-5.0);
    rightTalon.Set(-5.0);
    ClimberState = false;
}

bool ClimberSubsystem::IsExtended() {
    return ClimberState;
}


// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}
