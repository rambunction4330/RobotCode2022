// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "rmb/math/misc.h"
#include "rmb/io/log.h"

#include "rmb/math/trajectory/trajectory.h"
#include "shooter/ShootCommand.h"
#include <iostream>

ShootCommand::ShootCommand(
        TurretSubsystem& turret,
        ShooterSubsystem& shooter,
        HoodSubsystem& hood,
        DriveSubsystem& drivetrain,
        StorageSubsystem& storage,
        const VisionSubsystem& vision
): turretSubsystem(turret), shooterSubsystem(shooter), hoodSubsystem(hood), driveSubsystem(drivetrain), storageSubsystem(storage),
visionSubsystem(vision){
    AddRequirements({&turret, &shooter, &hood, &drivetrain, &storageSubsystem});
}

// Called when the command is initially scheduled.
void ShootCommand::Initialize() {
    turretPosition = 0.0_rad;
    storageTimer.Stop();
    storageTimer.Reset();
    isCanceled = false;
    wpi::outs() << "ShootCommand Initialized!" << wpi::endl;
    if(!storageSubsystem.hasBall()) {
        wpi::outs() << "Storage subsystem has no ball!" << wpi::endl;
        isCanceled = true;

    }

    if(!rmb::withinRange(visionSubsystem.getAngleToHub() - turretSubsystem.getAngularPosition(), -turretAngleErrorMargin, turretAngleErrorMargin)) {
        wpi::outs() << "Not aligned with hub!" << wpi::endl;
        isCanceled = true;
    } else {
        turretPosition = turretSubsystem.getAngularPosition();
        return;
    }
    wpi::outs() << "turret position: " << (units::degree_t) turretSubsystem.getAngularPosition() << wpi::endl;
    isCanceled = true;
}

// Called repeatedly when this Command is scheduled to run
void ShootCommand::Execute() {
    std::cout << "out test!";
    wpi::outs() << wpi::flush;

    driveSubsystem.stop();
    turretSubsystem.spinTo(turretPosition);
    //wpi::outs() << "turret position: " << (units::degree_t) turretPosition << wpi::endl;

    units::meter_t px = visionSubsystem.getHubHorizontalPos();
    units::meter_t py = visionSubsystem.getHubHeight();
    units::radian_t entryAngle = 30_deg;
    rmb::trajectory::Trajectory trajectory = rmb::trajectory::trajectoryFromEntryAngle(px, py, entryAngle);

    trajectory.angle = 90_deg - (units::degree_t)trajectory.angle;
    trajectory.velocity *= 3.0;

    //wpi::outs() << "TGT VELOCITY: " << trajectory.velocity << " TGT ANGLE: " << (units::degree_t)trajectory.angle << wpi::endl;
    //wpi::outs() << "Actual Velocity: " << shooterSubsystem.getVelocity() << wpi::endl;
    wpi::outs() << "TGT VELOCITY: " << trajectory.velocity << " -> " << shooterSubsystem.getVelocity() << wpi::endl;

    if(!rmb::withinRange((units::angle::degree_t)trajectory.angle, 21.0_deg, 42_deg)) {
        wpi::outs() << "target angle out of range!" << wpi::endl;
        return;
    }

    hoodSubsystem.setPosition(trajectory.angle);
    wpi::outs() << "hood position: " << (units::degree_t)hoodSubsystem.getPosition()
        << " target: " << (units::degree_t) trajectory.angle << wpi::endl;
    if(!hoodSubsystem.isAtPosition(trajectory.angle)) {
        wpi::outs() << "hood angle out of range" << wpi::endl;
        return;
    }

    shooterSubsystem.spinTo(trajectory.velocity);

    if(!rmb::withinRange(shooterSubsystem.getVelocity(), trajectory.velocity - 3.0_mps, trajectory.velocity + 3.0_mps)) {
        wpi::outs() << "velocity out of range!" << wpi::endl;
        return;
    }

    if(storageTimer.Get() == 0.0_s) {
        storageTimer.Start();
    }
    wpi::outs() << "spinning storage ... " << storageTimer.Get()() << "_s";
    storageSubsystem.spinStorage(0.5);
}

// Called once the command ends or is interrupted.
void ShootCommand::End(bool interrupted) {
    storageSubsystem.stop();
    shooterSubsystem.stop();
    hoodSubsystem.setPosition(21_deg);
    storageTimer.Stop();
}

// Returns true when the command should end.
bool ShootCommand::IsFinished() {
  return storageTimer.HasElapsed(1.0_s) || isCanceled;
}
