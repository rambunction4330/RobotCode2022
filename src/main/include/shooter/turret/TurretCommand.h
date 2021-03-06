// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <vision/VisionSubsystem.h>
#include <shooter/turret/TurretSubsystem.h>

/**
 * Makes the turret find and point to the hub using vision
 */
class TurretCommand
    : public frc2::CommandHelper<frc2::CommandBase, TurretCommand> {
 public:

  /**
   * Creates a TurretFindCommand
   * @param turretSubsystem the turret subsystem of the robot used for moving the shooter and the vision subsystem towards the target
   * @param visionSubsystem the visionSubsystem used to tell if the hub is in view
   */
  TurretCommand(TurretSubsystem& turretSubsystem, const VisionSubsystem& visionSubsystem);

  /**
   * Called at the beginning when the command is scheduled. 
   */
  void Initialize() override;

  /**
   * Called in a loop on the command scheduler. Spins around until it finds the hub.
   */
  void Execute() override;

  /**
   * Called when IsFinished() returns true or when the command is interrupted. 
   */
  void End(bool interrupted) override;

  /**
   * Checks if the command is finished or not
   * @return true if the hub is in sight
   */
  bool IsFinished() override;

private:
  TurretSubsystem& turretSubsystem;
  const VisionSubsystem& visionSubsystem;
  units::scalar_t spinDirection = 1.0;
};
