// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <shooter/TurretFollowCommand.h>
#include "shooter/TurretFindCommand.h"
#include "frc2/command/CommandScheduler.h"

TurretFindCommand::TurretFindCommand(TurretSubsystem& turret, VisionSubsystem& vision, TurretFollowCommand& followCommand)
  : turretSubsystem(turret), visionSubsystem(vision), turretFollowCommand(followCommand) {
  AddRequirements({&turret, &vision});
}

// Called when the command is initially scheduled.
void TurretFindCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TurretFindCommand::Execute() {
  if(!IsFinished()) {
    turretSubsystem.sweep();
  }
}

// Called once the command ends or is interrupted.
void TurretFindCommand::End(bool interrupted) {
  if(!interrupted) {
    turretFollowCommand.Schedule(true);
  }
}

// Returns true when the command should end.
bool TurretFindCommand::IsFinished() {
  return visionSubsystem.IsHubInView();
}
