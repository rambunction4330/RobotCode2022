// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "drivetrain/DriveSubsystem.h"

DriveSubsystem::DriveSubsystem()
    : frontLeft(-1), frontRight(-1), rearLeft(-1), rearRight(-1),
      kinematics(frc::Translation2d(), frc::Translation2d(),
                 frc::Translation2d(), frc::Translation2d()),
      drive(frontLeft, frontRight, rearLeft, rearRight, kinematics,
            units::meters_per_second_t(0), units::radians_per_second_t(0)),
      gyro(frc::SPI::kMXP), odometry(drive, gyro) {}

// This method will be called once per scheduler run
void DriveSubsystem::Periodic() { odometry.updatePose(); }
