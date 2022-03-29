// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "intake/IntakeIntakeCommand.h"
#include "intake/IntakeExtenderSubsystem.h"
#include "intake/IntakeSpinnerSubsystem.h"

IntakeIntakeCommand::IntakeIntakeCommand(IntakeExtenderSubsystem& extender, IntakeSpinnerSubsystem& spinner, StorageSubsystem& storage)
  : intakeExtenderSubsystem(extender), intakeSpinnerSubsystem(spinner), storageSubsystem(storage) {
  AddRequirements({&intakeExtenderSubsystem, &intakeSpinnerSubsystem, &storageSubsystem});
}

// Called when the command is initially scheduled.
void IntakeIntakeCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void IntakeIntakeCommand::Execute() {
   intakeExtenderSubsystem.extend();
    if (intakeExtenderSubsystem.isExtended()) {
      intakeSpinnerSubsystem.spin(1.0);
      storageSubsystem.spinStorage(0.5);
    }
}

// Called once the command ends or is interrupted.
void IntakeIntakeCommand::End(bool interrupted) {
     if (intakeExtenderSubsystem.isExtended()) {
      storageSubsystem.spinStorage(-0.2);
    } else {
      storageSubsystem.stop();
    }

    intakeSpinnerSubsystem.stop();
    intakeExtenderSubsystem.retract();
}

// Returns true when the command should end.
bool IntakeIntakeCommand::IsFinished() {
  return storageSubsystem.hasBall();
}
