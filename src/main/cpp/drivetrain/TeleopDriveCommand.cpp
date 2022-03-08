// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "drivetrain/TeleopDriveCommand.h"
#include <rmb/io/log.h>

TeleopDriveCommand::TeleopDriveCommand(DriveSubsystem &drvSubsys,
                                       const JoystickSubsystem &joySubsys)
    : driveSubsystem{drvSubsys}, joystickSubsystem{joySubsys} {
  AddRequirements(&driveSubsystem);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void TeleopDriveCommand::Initialize() {
  wpi::outs() << "Teleop initialized!" << wpi::endl;
}

// Called repeatedly when this Command is scheduled to run
void TeleopDriveCommand::Execute() {
  double throttle = joystickSubsystem.getThrottle();
  driveSubsystem.driveCartesian(joystickSubsystem.getY() * throttle,
                                joystickSubsystem.getX() * throttle,
                                joystickSubsystem.getTwist() * throttle);
}

// Called once the command ends or is interrupted.
void TeleopDriveCommand::End(bool interrupted) {
  if (!interrupted) {
    wpi::outs() << "Command exited cleanly!" << wpi::endl;
  } else {
    wpi::outs() << "Command was interrupted..." << wpi::endl;
  }
}

// Returns true when the command should end.
bool TeleopDriveCommand::IsFinished() { return false; }
