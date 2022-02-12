// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <shooter/TurretFindCommand.h>
#include "shooter/TurretFollowCommand.h"  
#include <frc2/command/CommandScheduler.h>

TurretFollowCommand::TurretFollowCommand(TurretSubsystem& turret, VisionSubsystem& vision) 
  : turretSubsystem(turret), visionSubsystem(vision), turretFindCommand(new TurretFindCommand(turret, vision, *this)){
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({&turretSubsystem, &visionSubsystem});
}

// Called when the command is initially scheduled.
void TurretFollowCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TurretFollowCommand::Execute() {
  if(!IsFinished()) {
    turretSubsystem.spinTo(visionSubsystem.getAngleToHub());
  }
}

// Called once the command ends or is interrupted.
void TurretFollowCommand::End(bool interrupted) {
  if(!interrupted) {
    turretFindCommand->Schedule(true);
  }
}

// Returns true when the command should end.
bool TurretFollowCommand::IsFinished() {
  return !visionSubsystem.IsHubInView();
}