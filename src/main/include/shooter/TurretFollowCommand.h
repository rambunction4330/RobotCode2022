// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <shooter/TurretSubsystem.h>
#include <vision/VisionSubsystem.h>

class TurretFindCommand;

/**
 * Follows the hub
 */
class TurretFollowCommand
    : public frc2::CommandHelper<frc2::CommandBase, TurretFollowCommand> {
 public:

  /**
   * Creates a TurretFollowCommand
   * @param turretSubsystem the turretSubsystem that will spin towards the target
   * @param visionSubsystem the visionSubsystem that will track the target
   */
  TurretFollowCommand(TurretSubsystem& turretSubsystem, VisionSubsystem& visionSubsystem);

  /**
   * Called when the command is ran by the CommandScheduler
   */
  void Initialize() override;

  /**
   * Called in a loop by the command scheduler. The turret will follow the Hub using vision.
   */
  void Execute() override;

  /**
   * Either called when IsFinished() returns true or the command is interrupted.
   */
  void End(bool interrupted) override;

  /**
   * Whether or not the command is finished or not
   * @return true if the hub is out of sight, which means the command is finshed.
   */
  bool IsFinished() override;

private:
  TurretSubsystem& turretSubsystem;
  VisionSubsystem& visionSubsystem;
  std::unique_ptr<TurretFindCommand> turretFindCommand;
};
