// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/HolonomicDriveController.h>
#include <frc/geometry/Pose2d.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <rmb/drive/DriveOdometry.h>
#include <rmb/drive/HolonomicDrive.h>

namespace rmb {
class HolonomicPointCommand
    : public frc2::CommandHelper<frc2::CommandBase, HolonomicPointCommand> {
public:
  HolonomicPointCommand(const frc::Pose2d &pose, HolonomicDrive &drive,
                        const DriveOdometry &odometry,
                        frc::HolonomicDriveController &controller);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  frc::Pose2d pose;
  HolonomicDrive &drive;
  const DriveOdometry &odometry;
  frc::HolonomicDriveController &controller;
};
} // namespace rmb