// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "autonomus/SingleBallAutoCommand.h"

SingleBallAutoCommand::SingleBallAutoCommand() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SingleBallAutoCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SingleBallAutoCommand::Execute() {}

// Called once the command ends or is interrupted.
void SingleBallAutoCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool SingleBallAutoCommand::IsFinished() {
  return false;
}
