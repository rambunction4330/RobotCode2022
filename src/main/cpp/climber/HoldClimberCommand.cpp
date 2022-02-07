// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "climber/HoldClimberCommand.h"

HoldClimberCommand::HoldClimberCommand() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void HoldClimberCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void HoldClimberCommand::Execute() {}

// Called once the command ends or is interrupted.
void HoldClimberCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool HoldClimberCommand::IsFinished() {
  return false;
}
