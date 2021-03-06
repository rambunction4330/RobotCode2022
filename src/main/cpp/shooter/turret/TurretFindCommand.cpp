// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <shooter/turret/TurretFollowCommand.h>
#include "shooter/turret/TurretFindCommand.h"
#include "frc2/command/CommandScheduler.h"

TurretFindCommand::TurretFindCommand(TurretSubsystem& turret, const VisionSubsystem& vision)
  : turretSubsystem(turret), visionSubsystem(vision) {
  AddRequirements({&turret});
}

// Called when the command is initially scheduled.
void TurretFindCommand::Initialize() {
    if(turretSubsystem.getAngularPosition() < 0_tr) {
        spinDirection = -1;
    } else {
        spinDirection = 1;
    }
}

// Called repeatedly when this Command is scheduled to run
void TurretFindCommand::Execute() {
    turretSubsystem.spinTo(spinDirection < 0.0 ? turretSubsystemConstants::minPosition : turretSubsystemConstants::maxPosition);

    if(turretSubsystem.isAtPosition(turretSubsystemConstants::minPosition) || turretSubsystem.isAtPosition(turretSubsystemConstants::maxPosition)) {
        spinDirection *= -1;
    }
}

// Called once the command ends or is interrupted.
void TurretFindCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool TurretFindCommand::IsFinished() {
  return visionSubsystem.IsHubInView();
}
