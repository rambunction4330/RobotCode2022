// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "drivetrain/DriveSubsystem.h"


DriveSubsystem::DriveSubsystem()
    : frontLeft(driveSubsystemConstants::frontLeftID, driveSubsystemConstants::motorPIDConfig, driveSubsystemConstants::motorConvertion, driveSubsystemConstants::motorFeedforward), 
      frontRight(driveSubsystemConstants::frontRightID, driveSubsystemConstants::motorPIDConfig, driveSubsystemConstants::motorConvertion, driveSubsystemConstants::motorFeedforward), 
      rearLeft(driveSubsystemConstants::rearLeftID, driveSubsystemConstants::motorPIDConfig, driveSubsystemConstants::motorConvertion, driveSubsystemConstants::motorFeedforward), 
      rearRight(driveSubsystemConstants::rearRightID, driveSubsystemConstants::motorPIDConfig, driveSubsystemConstants::motorConvertion, driveSubsystemConstants::motorFeedforward),
      kinematics(driveSubsystemConstants::frontLeftPose, 
                 driveSubsystemConstants::frontRightPose,
                 driveSubsystemConstants::rearLeftPose, 
                 driveSubsystemConstants::rearRightPose),
      drive(frontLeft, frontRight, rearLeft, rearRight, kinematics,
            driveSubsystemConstants::maxVelocity, driveSubsystemConstants::maxRotVelocity),
      gyro(driveSubsystemConstants::gyroPort), odometry(drive, gyro) {}

// This method will be called once per scheduler run
void DriveSubsystem::Periodic() { odometry.updatePose(); }
