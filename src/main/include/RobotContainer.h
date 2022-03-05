// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>

#include "driverstation/JoystickSubsystem.h"
#include "drivetrain/DriveSubsystem.h"
#include "drivetrain/TeleopDriveCommand.h"
#include "driverstation/ShuffleBoardSubsystem.h"

#include "shooter/turret/TurretSubsystem.h"
#include "shooter/turret/TurretFindCommand.h"
#include "shooter/turret/TurretFollowCommand.h"
#include "shooter/ManualShooterCommand.h"
/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
public:
  RobotContainer();

  TeleopDriveCommand &getTeleopDriveCommand() { return teleopDriveCommand; }
  //ManualShooterCommand& getManualShooterCommand() { return manualShooterCommand; }
private:

  void ConfigureButtonBindings();
  void InitializeTurret();

  ShooterSubsystem  shooterSubsystem{};
  TurretSubsystem   turretSubsystem{};

  //ClimberSubsystem  climberSubsystem;
  DriveSubsystem    driveSubsystem;
  IntakeSubsystem   intakeSubsystem;
  JoystickSubsystem joystickSubsystem{0};
  VisionSubsystem   visionSubsystem{ turretSubsystem};
  //TeleopDriveCommand teleopDriveCommand{ driveSubsystem, joystickSubsystem };
  //TurretFindCommand turretFindCommand{ turretSubsystem, visionSubsystem };
  //ShuffleBoardSubsystem  shuffleBoard{ shooterSubsystem, joystickSubsystem, climberSubsystem, driveSubsystem, intakeSubsystem };
  //ManualShooterCommand manualShooterCommand{ shooterSubsystem, joystickSubsystem, turretSubsystem };
};
