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
#include "storage/StorageSubsystem.h"

#include <frc/trajectory/TrajectoryGenerator.h>

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
  ManualShooterCommand* getManualShooterCommand() { return &manualShooterCommand; }
  std::unique_ptr<frc2::Command> getTrajectoryCommand() {

      static frc::Pose2d start {
        0.0_m, 0.0_m, 0.0_deg
      };

      static frc::Pose2d end {
          0.0_m, 0.0_m, 0.0_deg
      };


//      static std::vector<frc::Translation2d> waypoints {
//          frc::Translation2d{0.0_m, 1.0_m},
//          frc::Translation2d{1.0_m, 0.0_m},
//          frc::Translation2d{0.0_m, -1.0_m}
//      };

      static std::vector<frc::Pose2d> waypoints {
          frc::Pose2d{0.0_m, 0.0_m, 0.0_rad},
          frc::Pose2d{0.0_m, 1.0_m, 0.0_rad}
      };

      static frc::TrajectoryConfig trajectoryConfig {
          2.5_mps,
          5_mps_sq
      };

      static
      auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
              waypoints,
              trajectoryConfig
      );
      return driveSubsystem.generateTrajectoryCommand(trajectory);
  }

private:

  void ConfigureButtonBindings();
  void InitializeTurret();

  ShooterSubsystem  shooterSubsystem{};
  TurretSubsystem   turretSubsystem{};
//
//  ClimberSubsystem  climberSubsystem;
  // ShooterSubsystem  shooterSubsystem;
  // ClimberSubsystem  climberSubsystem;
  DriveSubsystem    driveSubsystem;

  IntakeExtenderSubsystem intakeExtenderSubsystem;
  IntakeSpinnerSubsystem intakeSpinnerSubsystem{intakeExtenderSubsystem};
  StorageSubsystem storageSubsystem;
  JoystickSubsystem joystickSubsystem{0, 0.2, {-1.0, -1.0, -1.0}};
  JoystickSubsystem turretJoystickSubsystem{1, 0.2, {-1.0, -1.0, 1.0}};
  TeleopDriveCommand teleopDriveCommand{ driveSubsystem, joystickSubsystem };
  HoodSubsystem hoodSubsystem{};
//  ShuffleBoardSubsystem  shuffleBoard{ shooterSubsystem, joystickSubsystem, climberSubsystem, driveSubsystem, intakeExtenderSubsystem, intakeSpinnerSubsystem };
//  VisionSubsystem   visionSubsystem{ turretSubsystem};
//  TurretFindCommand turretFindCommand{ turretSubsystem, visionSubsystem };
  ManualShooterCommand manualShooterCommand{ shooterSubsystem, turretJoystickSubsystem, turretSubsystem, storageSubsystem, hoodSubsystem };

  // BEGIN Holonomic Trajectory Tests

  // END

};
