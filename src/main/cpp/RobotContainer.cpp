// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "frc2/command/ConditionalCommand.h"

#include <frc2/command/RunCommand.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  intakeExtenderSubsystem.SetDefaultCommand(frc2::RunCommand([&]() { intakeExtenderSubsystem.retract(); }, {&intakeExtenderSubsystem}));
  intakeSpinnerSubsystem.SetDefaultCommand(frc2::RunCommand([&]() { intakeSpinnerSubsystem.stop(); }, {&intakeSpinnerSubsystem}));
  storageSubsystem.SetDefaultCommand(frc2::RunCommand([&]() { storageSubsystem.stop(); }, {&storageSubsystem}) /*frc2::ConditionalCommand(storageSubsystem.spinStorageCommand(0.5), storageSubsystem.stopCommand(), [&]() { return intakeSpinnerSubsystem.isSpinning(); })*/);
  // Configure the button bindings
  ConfigureButtonBindings();

  // Initialize turret
  InitializeTurret();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
    // Extend Intake
  joystickSubsystem.getButton(11).ToggleWhenPressed(frc2::FunctionalCommand([&]() {}, [&]() {
    intakeExtenderSubsystem.extend(); 
  }, [&](bool) {
    intakeExtenderSubsystem.retract();
  }, [&]() {
    return storageSubsystem.hasBall();
  }, {&intakeExtenderSubsystem}));

  // Pull balls in
  joystickSubsystem.getButton(12).ToggleWhenPressed(frc2::FunctionalCommand([&]() {}, [&]() {
    intakeSpinnerSubsystem.spin(1.0);  
    storageSubsystem.spinStorage(0.5);
  }, [&](bool a) {
    intakeSpinnerSubsystem.stop(); 
    storageSubsystem.spinStorage(-1.0);
  }, [&]() {
    return !intakeExtenderSubsystem.isExtended() || storageSubsystem.hasBall();
  }, {&intakeSpinnerSubsystem, &storageSubsystem}));

  // Spit balls out
  joystickSubsystem.getButton(9).WhileHeld([&]() {
    intakeExtenderSubsystem.extend();
    intakeSpinnerSubsystem.spin(-0.5);
    storageSubsystem.spinStorage(-1.0);
  }, {&intakeExtenderSubsystem, &intakeSpinnerSubsystem, &storageSubsystem});
}

void RobotContainer::InitializeTurret() {
//  turretSubsystem.SetDefaultCommand(
//    frc2::ConditionalCommand(
//      TurretFollowCommand(turretSubsystem, visionSubsystem),
//      TurretFindCommand(turretSubsystem, visionSubsystem),
//      [&]() { return visionSubsystem.IsHubInView(); })
//    );
}
