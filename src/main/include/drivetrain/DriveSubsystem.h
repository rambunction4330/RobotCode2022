// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <units/length.h>

#include <frc/SPI.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc2/command/Command.h>

#include <rmb/drive/MecanumDrive.h>
#include <rmb/drive/MecanumEncoderOdometry.h>
#include <rmb/motorcontrol/sparkmax/SparkMaxVelocityController.h>

#include "Constants.h"

class DriveSubsystem : public frc2::SubsystemBase {
public:
  DriveSubsystem();

  void Periodic() override;

  void driveCartesian(double ySpeed, double xSpeed, double rotation, bool fieldOriented = false, bool rotationCorrection = false);
  void drivePolar(units::radians direction, double magnitude, double rotation);

  std::unique_ptr<frc2::Command> generatePointCommand(frc::Pose2d point);
  std::unique_ptr<frc2::Command> generateTrajectoryCommand(frc::Trajectory trajectory);
  std::unique_ptr<frc2::Command> generateDanceCommand();

  const frc::Pose2d& getPosition() const;
  units::radian_t getGyroHeading() const;
  frc::ChassisSpeeds getChassisSpeeds() const;
  frc::ChassisSpeeds getFieldRelativeSpeeds() const;

  void resetGyro(units::radian_t angle = 0.0_rad);
  void resetPosition(frc::Pose2d position = frc::Pose2d());

private:
  rmb::SparkMaxVelocityController<units::meters> frontLeft;
  rmb::SparkMaxVelocityController<units::meters> frontRight;
  rmb::SparkMaxVelocityController<units::meters> rearLeft;
  rmb::SparkMaxVelocityController<units::meters> rearRight;
  frc::MecanumDriveKinematics kinematics;
  rmb::MecanumDrive drive;
  AHRS gyro;
  rmb::MecanumEncoderOdometry odometry;
  frc::HolonomicDriveController driveContoller;
};
