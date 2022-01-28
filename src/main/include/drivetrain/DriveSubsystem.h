// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/SPI.h>
#include <frc2/command/SubsystemBase.h>
#include <units/length.h>

#include <rmb/drive/MecanumDrive.h>
#include <rmb/drive/MecanumEncoderOdometry.h>
#include <rmb/motorcontrol/SparkMax/SparkMaxVelocityController.h>

#include "Constants.h"

class DriveSubsystem : public frc2::SubsystemBase {
public:
  DriveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

private:
  rmb::SparkMaxVelocityController<units::meters> frontLeft;
  rmb::SparkMaxVelocityController<units::meters> frontRight;
  rmb::SparkMaxVelocityController<units::meters> rearLeft;
  rmb::SparkMaxVelocityController<units::meters> rearRight;
  frc::MecanumDriveKinematics kinematics;
  rmb::MecanumDrive drive;
  AHRS gyro;
  rmb::MecanumEncoderOdometry odometry;
};
