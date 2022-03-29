// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <drivetrain/DriveSubsystem.h>
#include <vision/VisionSubsystem.h>
#include <shooter/turret/TurretSubsystem.h>
#include <shooter/ShooterSubsystem.h>
#include <shooter/hood/HoodSubsystem.h>
#include <storage/StorageSubsystem.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ShootCommand
    : public frc2::CommandHelper<frc2::CommandBase, ShootCommand> {
 public:
  ShootCommand(
          TurretSubsystem& turret,
          ShooterSubsystem& shooter,
          HoodSubsystem& hood,
          DriveSubsystem& drive,
          StorageSubsystem& storage,
          const VisionSubsystem& vision
      );

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
private:

    const units::radian_t turretAngleErrorMargin = 10.0_deg; /**< Has to be between positive and negative this value */

    frc::Timer storageTimer {};

    TurretSubsystem& turretSubsystem;
    ShooterSubsystem& shooterSubsystem;
    HoodSubsystem& hoodSubsystem;
    DriveSubsystem& driveSubsystem;
    StorageSubsystem& storageSubsystem;
    const VisionSubsystem& visionSubsystem;

    units::radian_t turretPosition = 0.0_rad;
    bool isCanceled = false;
};
