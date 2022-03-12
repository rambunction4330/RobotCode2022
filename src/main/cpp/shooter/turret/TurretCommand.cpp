#include <shooter/turret/TurretCommand.h>
#include "frc2/command/CommandScheduler.h"

TurretCommand::TurretCommand(TurretSubsystem& turret, const VisionSubsystem& vision)
        : turretSubsystem(turret), visionSubsystem(vision) {
    AddRequirements({&turret});
}

// Called when the command is initially scheduled.
void TurretCommand::Initialize() {
    if(turretSubsystem.getAngularPosition() < 0_tr) {
        spinDirection = -1;
    } else {
        spinDirection = 1;
    }
}

// Called repeatedly when this Command is scheduled to run
void TurretCommand::Execute() {
    if(!visionSubsystem.IsHubInView()) {
        turretSubsystem.spinTo(spinDirection < 0.0 ? turretSubsystemConstants::minPosition : turretSubsystemConstants::maxPosition);

        if(turretSubsystem.isAtPosition(turretSubsystemConstants::minPosition) || turretSubsystem.isAtPosition(turretSubsystemConstants::maxPosition)) {
            spinDirection *= -1;
        }
    } else {
        turretSubsystem.spinOffset(visionSubsystem.getAngleToHub());
    }
}

// Called once the command ends or is interrupted.
void TurretCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool TurretCommand::IsFinished() {
    return false;
}