// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "frc2/command/ConditionalCommand.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  // Initialize turret
  InitializeTurret();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

void RobotContainer::InitializeTurret() {
//  turretSubsystem.SetDefaultCommand(
//    frc2::ConditionalCommand(
//      TurretFollowCommand(turretSubsystem, visionSubsystem),
//      TurretFindCommand(turretSubsystem, visionSubsystem),
//      [&]() { return visionSubsystem.IsHubInView(); })
//    );
}
