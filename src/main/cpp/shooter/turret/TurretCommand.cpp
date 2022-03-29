#include <shooter/turret/TurretCommand.h>
#include "frc2/command/CommandScheduler.h"
#include <rmb/io/log.h>

TurretCommand::TurretCommand(TurretSubsystem& turret, const VisionSubsystem& vision)
        : turretSubsystem(turret), visionSubsystem(vision) {

    AddRequirements({&turret});
}

// Called when the command is initially scheduled.
void TurretCommand::Initialize() {
    //wpi::outs() << "Turret command init!" << wpi::endl;
    if(turretSubsystem.getAngularPosition() < 0_tr) {
        spinDirection = -1;
    } else {
        spinDirection = 1;
    }
}

// Called repeatedly when this Command is scheduled to run
void TurretCommand::Execute() {
    //wpi::outs() << "turret command execute" << wpi::endl;
    //wpi::outs() << "hub " << (visionSubsystem.IsHubInView() ? "is " : "is not ") << "in view" << wpi::endl;
    if(!visionSubsystem.IsHubInView()) {
        turretSubsystem.spinTo(spinDirection < 0.0 ? turretSubsystemConstants::minPosition : turretSubsystemConstants::maxPosition);

        if(turretSubsystem.isAtPosition(turretSubsystemConstants::minPosition) || turretSubsystem.isAtPosition(turretSubsystemConstants::maxPosition)) {
            spinDirection *= -1;
        }
    } else {
        static units::radian_t lastOffset = 0.0_rad;

        units::radian_t offset = visionSubsystem.getAngleToHub();

        if(lastOffset != offset) {
            turretSubsystem.spinTo(offset);
            lastOffset = visionSubsystem.getAngleToHub();
        }

        //wpi::outs() << "angle to hub: " << visionSubsystem.getAngleToHub() << wpi::endl;
    }
}

// Called once the command ends or is interrupted.
void TurretCommand::End(bool interrupted) {
    //wpi::outs() << "turret command end!" << wpi::endl;
}

// Returns true when the command should end.
bool TurretCommand::IsFinished() {
    return false;
}