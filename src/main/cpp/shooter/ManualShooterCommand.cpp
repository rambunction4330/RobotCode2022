// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <rmb/io/log.h>
#include "shooter/ManualShooterCommand.h"

ManualShooterCommand::ManualShooterCommand(ShooterSubsystem& shooter, JoystickSubsystem& joystick, TurretSubsystem& turret, StorageSubsystem& storage) :
   shooterSubsystem(shooter), joystickSubsystem(joystick), turretSubsystem(turret), storageSubsystem(storage) {
  AddRequirements({&shooterSubsystem, &joystickSubsystem, &turretSubsystem});
}

// Called when the command is initially scheduled.
void ManualShooterCommand::Initialize() {
  wpi::outs() << "Command Schedueled!" << wpi::endl;
}

// Called repeatedly when this Command is scheduled to run
void ManualShooterCommand::Execute() {
    double twist = joystickSubsystem.getTwist();
    wpi::outs() << "turret -> " << twist << wpi::endl;
    turretSubsystem.spinOffset(
            twist > 0 ?
                turretSubsystemConstants::maxPosition * std::abs(twist) :
                turretSubsystemConstants::minPosition * std::abs(twist)
    );
}

// Called once the command ends or is interrupted.
void ManualShooterCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool ManualShooterCommand::IsFinished() {
  return false;
}

