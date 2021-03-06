// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>

#include "driverstation/JoystickSubsystem.h"
#include "drivetrain/DriveSubsystem.h"
#include "drivetrain/TeleopDriveCommand.h"
#include "driverstation/ShuffleBoardSubsystem.h"

#include "intake/IntakeIntakeCommand.h"
#include "shooter/ShootCommand.h"
#include "shooter/turret/TurretSubsystem.h"

#include "shooter/turret/TurretCommand.h"

#include "shooter/ManualShooterCommand.h"
#include "storage/StorageSubsystem.h"

#include <frc/trajectory/TrajectoryGenerator.h>
#include <memory>

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

  void resetOdometry() {
      driveSubsystem.resetPosition();
      driveSubsystem.resetGyro();
  }

  TeleopDriveCommand &getTeleopDriveCommand() { return teleopDriveCommand; }
  ManualShooterCommand* getManualShooterCommand() { return &manualShooterCommand; }
  std::unique_ptr<frc2::Command> getAutonomousTrajectoryCommand() {

      driveSubsystem.resetGyro(0.0_rad);
      static frc::Pose2d start {
        0.0_m, 0.0_m, 0.0_deg
      };

      static frc::Pose2d end {
          1.0_m, 1.0_m, 0.0_deg
      };


      static std::vector<frc::Pose2d> waypoints {
          frc::Pose2d{0.0_ft, 0.0_ft, 0.0_rad},
          frc::Pose2d{6.0_ft, 0.0_ft, 0.0_rad},
//          frc::Pose2d{1.75_m, 0.0_m, 0.0_rad},
//          frc::Pose2d{2.5_m, -1.0_m, 0.0_rad},
//          frc::Pose2d{3.250_m, 0.0_m, 0.0_rad},
//          frc::Pose2d{4.0_m, 1.0_m, 0.0_rad},
//          frc::Pose2d{5.0_m, 0.0_m, 0.0_rad}
          };

//      static std::vector<frc::Pose2d> waypoints {
//          frc::Pose2d{0.0_m, 0.0_m, 0.0_rad},
//          frc::Pose2d{1.0_m, 0.0_m, 0.0_rad}
//      };

      static frc::TrajectoryConfig trajectoryConfig {
          1_mps,
          1_mps_sq
      };

      static
      auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
              waypoints,
              trajectoryConfig
      );
      return driveSubsystem.generateTrajectoryCommand(trajectory);
  }

  std::unique_ptr<frc2::Command> getInatkeCommand() {
    return std::unique_ptr<frc2::Command>(new IntakeIntakeCommand(intakeExtenderSubsystem, intakeSpinnerSubsystem, storageSubsystem));
  }

  ShootCommand getShootCommand() {
    return ShootCommand(
                  turretSubsystem,
                  shooterSubsystem,
                  hoodSubsystem,
                  driveSubsystem,
                  storageSubsystem,
                  visionSubsystem
                  );
  }
private:

  void ConfigureButtonBindings();
  void InitializeTurret();

  ShooterSubsystem  shooterSubsystem{};
  TurretSubsystem   turretSubsystem{};
//
//  ClimberSubsystem  climberSubsystem;
  // ShooterSubsystem  shooterSubsystem;
  ClimberSubsystem  climberSubsystem;
  DriveSubsystem    driveSubsystem;

  IntakeExtenderSubsystem intakeExtenderSubsystem;
  IntakeSpinnerSubsystem intakeSpinnerSubsystem{intakeExtenderSubsystem};
  StorageSubsystem storageSubsystem;
  JoystickSubsystem joystickSubsystem{0, 0.3, {-1.0, -1.0, 1.0}};
  JoystickSubsystem turretJoystickSubsystem{1, 0.2, {-1.0, -1.0, 1.0}};
  TeleopDriveCommand teleopDriveCommand{ driveSubsystem, joystickSubsystem };
  HoodSubsystem hoodSubsystem{};
//  ShuffleBoardSubsystem  shuffleBoard{ shooterSubsystem, joystickSubsystem, climberSubsystem, driveSubsystem, intakeExtenderSubsystem, intakeSpinnerSubsystem };
  VisionSubsystem   visionSubsystem {turretSubsystem};
  ManualShooterCommand manualShooterCommand{ shooterSubsystem, turretJoystickSubsystem, turretSubsystem, storageSubsystem, hoodSubsystem };

  // BEGIN Holonomic Trajectory Tests

  // END

};
