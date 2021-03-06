// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "frc2/command/ConditionalCommand.h"
#include "intake/IntakeSpinnerSubsystem.h"

#include <frc2/command/RunCommand.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>

#include <rmb/command/RepeatingCommand.h>
#include <intake/IntakeIntakeCommand.h>
#include <shooter/ShootCommand.h>

#include <memory>

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  intakeExtenderSubsystem.SetDefaultCommand(frc2::RunCommand([&]() { intakeExtenderSubsystem.retract(); }, {&intakeExtenderSubsystem}));
  intakeSpinnerSubsystem.SetDefaultCommand(frc2::RunCommand([&]() { intakeSpinnerSubsystem.stop(); }, {&intakeSpinnerSubsystem}));

  storageSubsystem.SetDefaultCommand(frc2::RunCommand([&]() { storageSubsystem.stop(); }, {&storageSubsystem}) /*frc2::ConditionalCommand(storageSubsystem.spinStorageCommand(0.5), storageSubsystem.stopCommand(), [&]() { return intakeSpinnerSubsystem.isSpinning(); })*/);

  driveSubsystem.SetDefaultCommand(getTeleopDriveCommand());
  turretSubsystem.SetDefaultCommand(TurretCommand(turretSubsystem, visionSubsystem));

  hoodSubsystem.SetDefaultCommand(frc2::RunCommand([this](){hoodSubsystem.stop();}, {&hoodSubsystem}));
  shooterSubsystem.SetDefaultCommand(frc2::RunCommand([this]() { shooterSubsystem.stop(); }, {&shooterSubsystem}));
  climberSubsystem.SetDefaultCommand(frc2::RunCommand([&]() { climberSubsystem.stopArm(); climberSubsystem.winchStop(); }, {&climberSubsystem}));

  // Configure the button bindings
  ConfigureButtonBindings();

  // Initialize turret

  InitializeTurret();

}

void RobotContainer::InitializeTurret() {

}

void RobotContainer::ConfigureButtonBindings() {

  // Extend  and pull ball in
  joystickSubsystem.getButton(11).ToggleWhenPressed(IntakeIntakeCommand(intakeExtenderSubsystem, intakeSpinnerSubsystem, storageSubsystem));

  turretJoystickSubsystem.getButton(11).ToggleWhenActive(manualShooterCommand);

  joystickSubsystem.getButton(1).WhenPressed(
          ShootCommand(
                  turretSubsystem,
                  shooterSubsystem,
                  hoodSubsystem,
                  driveSubsystem,
                  storageSubsystem,
                  visionSubsystem
                  ).WithTimeout(6_s)
              );
  // arm up
  joystickSubsystem.getButton(6).WhileHeld(
          frc2::RunCommand(
                  [this](){
                     climberSubsystem.extendArm();
                  },
                  {&climberSubsystem}
                  )
          );

  joystickSubsystem.getButton(4).WhileHeld(
          frc2::RunCommand(
                  [this]() {
                     climberSubsystem.retractArm();
                  },
                  {&climberSubsystem}
                  )
          );

    joystickSubsystem.getButton(3).WhileHeld(
            frc2::RunCommand(
                    [this](){
                        climberSubsystem.winchTighten();
                    },
                    {&climberSubsystem}
            )
    );
}
