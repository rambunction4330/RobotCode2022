// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "drivetrain/DriveSubsystem.h"

DriveSubsystem::DriveSubsystem()
    : frontLeft(driveSubsystemConstants::frontLeftID), 
      frontRight(driveSubsystemConstants::frontRightID), 
      rearLeft(driveSubsystemConstants::rearLeftID), 
      rearRight(driveSubsystemConstants::rearRightID),
      kinematics(driveSubsystemConstants::frontLeftPose, 
                 driveSubsystemConstants::frontRightPose,
                 driveSubsystemConstants::rearLeftPose, 
                 driveSubsystemConstants::rearRightPose),
      drive(frontLeft, frontRight, rearLeft, rearRight, kinematics,
            driveSubsystemConstants::maxXVelocity, driveSubsystemConstants::maxXVelocity),
      gyro(driveSubsystemConstants::gyroPort), odometry(drive, gyro) {}

// This method will be called once per scheduler run
void DriveSubsystem::Periodic() { odometry.updatePose(); }
