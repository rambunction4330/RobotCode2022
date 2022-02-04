// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "rmb/drive/HolonomicPointCommand.h"

namespace rmb {
HolonomicPointCommand::HolonomicPointCommand(const frc::Pose2d& pose, HolonomicDrive& drive, const DriveOdometry& odometry, frc::HolonomicDriveController& controller) : 
                                             pose(pose), drive(drive), odometry(odometry), controller(controller) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void HolonomicPointCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void HolonomicPointCommand::Execute() {}

// Called once the command ends or is interrupted.
void HolonomicPointCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool HolonomicPointCommand::IsFinished() {
  return false;
}
}