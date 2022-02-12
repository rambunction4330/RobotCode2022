// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <vision/VisionSubsystem.h>
#include <shooter/TurretSubsystem.h>

//class TurretFollowCommand : public frc2::CommandHelper<frc2::CommandBase, TurretFollowCommand> {};

class TurretFollowCommand;

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TurretFindCommand
    : public frc2::CommandHelper<frc2::CommandBase, TurretFindCommand> {
 public:
  TurretFindCommand(TurretSubsystem& turretSubsystem, VisionSubsystem& visionSubsystem, TurretFollowCommand& followCommand);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  TurretSubsystem& turretSubsystem;
  VisionSubsystem& visionSubsystem;
  TurretFollowCommand& turretFollowCommand;
};
