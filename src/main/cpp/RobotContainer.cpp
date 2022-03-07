// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

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
}

void RobotContainer::ConfigureButtonBindings() {

  // Extend  and pull ball in
  joystickSubsystem.getButton(ELLEVEN).ToggleWhenPressed(frc2::FunctionalCommand([&]() {}, [&]() { 
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
}