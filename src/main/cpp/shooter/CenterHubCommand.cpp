// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "shooter/CenterHubCommand.h"

CenterHubCommand::CenterHubCommand() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void CenterHubCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void CenterHubCommand::Execute() {}

// Called once the command ends or is interrupted.
void CenterHubCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool CenterHubCommand::IsFinished() {
  return false;
}
