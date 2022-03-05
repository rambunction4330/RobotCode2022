// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <shooter/ShooterSubsystem.h>
#include <driverstation/JoystickSubsystem.h>
#include <shooter/turret/TurretSubsystem.h>
#include <storage/StorageSubsystem.h>

/**
 * Shoots based on user input from the joystick
 */
class ManualShooterCommand
    : public frc2::CommandHelper<frc2::CommandBase, ManualShooterCommand> {
 public:
  ManualShooterCommand(ShooterSubsystem& shooter, JoystickSubsystem& joystick, TurretSubsystem& turret, StorageSubsystem& storage);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:

  ShooterSubsystem& shooterSubsystem;
  JoystickSubsystem& joystickSubsystem;
  TurretSubsystem& turretSubsystem;
  StorageSubsystem& storageSubsystem;
};
