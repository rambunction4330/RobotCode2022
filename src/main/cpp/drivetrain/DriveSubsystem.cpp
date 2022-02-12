// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "drivetrain/DriveSubsystem.h"
#include <rmb/drive/HolonomicTrajectoryCommand.h>

#include <rmb/io/log.h>

DriveSubsystem::DriveSubsystem()
    : frontLeft(driveSubsystemConstants::frontLeftID,
                driveSubsystemConstants::motorPIDConfig,
                driveSubsystemConstants::motorConvertion,
                driveSubsystemConstants::motorFeedforward),
      frontRight(driveSubsystemConstants::frontRightID,
                 driveSubsystemConstants::motorPIDConfig,
                 driveSubsystemConstants::motorConvertion,
                 driveSubsystemConstants::motorFeedforward),
      rearLeft(driveSubsystemConstants::rearLeftID,
               driveSubsystemConstants::motorPIDConfig,
               driveSubsystemConstants::motorConvertion,
               driveSubsystemConstants::motorFeedforward),
      rearRight(driveSubsystemConstants::rearRightID,
                driveSubsystemConstants::motorPIDConfig,
                driveSubsystemConstants::motorConvertion,
                driveSubsystemConstants::motorFeedforward),
      kinematics(driveSubsystemConstants::frontLeftPose,
                 driveSubsystemConstants::frontRightPose,
                 driveSubsystemConstants::rearLeftPose,
                 driveSubsystemConstants::rearRightPose),
      drive(frontLeft, frontRight, rearLeft, rearRight, kinematics,
            driveSubsystemConstants::maxVelocity,
            driveSubsystemConstants::maxRotVelocity),
      gyro(driveSubsystemConstants::gyroPort), odometry(drive, gyro),
      driveContoller(driveSubsystemConstants::xController,
                     driveSubsystemConstants::yController,
                     driveSubsystemConstants::thetaController) {
        frontRight.setInverted(true);
        rearRight.setInverted(true);
      }

// This method will be called once per scheduler run
void DriveSubsystem::Periodic() { odometry.updatePose(); }

void DriveSubsystem::driveCartesian(double ySpeed, double xSpeed,
                                    double rotation, bool fieldOriented,
                                    bool rotationCorrection) {
  drive.driveCartesian(ySpeed, xSpeed, rotation);
}

void DriveSubsystem::drivePolar(double magnitude, units::radian_t direction,
                                double rotation) {
  drive.drivePolar(magnitude, direction, rotation);
}

std::unique_ptr<frc2::Command>
DriveSubsystem::generateTrajectoryCommand(frc::Trajectory trajectory) {
  return std::unique_ptr<frc2::Command>(new rmb::HolonomicTrajectoryCommand(
      trajectory, drive, odometry, driveContoller, {this}));
}