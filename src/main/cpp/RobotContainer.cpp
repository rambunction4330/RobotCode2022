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
  storageSubsystem.SetDefaultCommand(frc2::RunCommand([&]() { storageSubsystem.stop(); }, {&storageSubsystem}));
  extenderSubsystem.SetDefaultCommand(frc2::RunCommand([&]() { extenderSubsystem.stopArm(); }, {&extenderSubsystem}));
  climberSubsystem.SetDefaultCommand(frc2::RunCommand([&]() { climberSubsystem.stop(); }, {&climberSubsystem}));

  // Configure the button bindings
  ConfigureButtonBindings();

  // Initialize turret
  InitializeTurret();
}

void RobotContainer::ConfigureButtonBindings() {
  // Extend and pull ball in
  joystickSubsystem.getButton(11).ToggleWhenPressed(frc2::FunctionalCommand([&]() {}, [&]() { 
    intakeExtenderSubsystem.extend();
    if (intakeExtenderSubsystem.isExtended()) {
      intakeSpinnerSubsystem.spin(1.0);  
      storageSubsystem.spinStorage(0.5);
    }
  }, [&](bool) {
    
    if (intakeExtenderSubsystem.isExtended()) {
      storageSubsystem.spinStorage(-1.0);
    } else {
      storageSubsystem.stop();
    }

    intakeSpinnerSubsystem.stop();
    intakeExtenderSubsystem.retract();

  }, [&]() {
    return storageSubsystem.hasBall();
  }, {&intakeExtenderSubsystem, &intakeSpinnerSubsystem, &storageSubsystem}));

  // Spit balls out
  joystickSubsystem.getButton(12).WhileHeld([&]() {
    intakeExtenderSubsystem.extend();
    intakeSpinnerSubsystem.spin(-0.5);
    storageSubsystem.spinStorage(-1.0);
  }, {&intakeExtenderSubsystem, &intakeSpinnerSubsystem, &storageSubsystem});

  joystickSubsystem.getButton(8).WhileHeld([&]() {
    extenderSubsystem.extendArm();
  }, {&extenderSubsystem});
 
  joystickSubsystem.getButton(7).WhileHeld([&]() {
    extenderSubsystem.retractArm();
  }, {&extenderSubsystem});

  joystickSubsystem.getButton(10).WhileHeld([&]() {
    climberSubsystem.climb();
  }, {&climberSubsystem});
 
  joystickSubsystem.getButton(9).WhileHeld([&]() {
    climberSubsystem.lower();
  }, {&climberSubsystem});
}

void RobotContainer::InitializeTurret() {
//  turretSubsystem.SetDefaultCommand(
//    frc2::ConditionalCommand(
//      TurretFollowCommand(turretSubsystem, visionSubsystem),
//      TurretFindCommand(turretSubsystem, visionSubsystem),
//      [&]() { return visionSubsystem.IsHubInView(); })
//    );
}
