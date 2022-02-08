// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <units/angle.h>

#include <rmb/motorcontrol/sparkmax/SparkMaxVelocityController.h>

#include <Constants.h>

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:

  rmb::SparkMaxVelocityController<units::radians>::Follower rightShooterMotor {
    shooterSubsystemConstants::rightShooterMotorID,
    rev::CANSparkMax::MotorType::kBrushless,
    false
  };

  rmb::SparkMaxVelocityController<units::radians> leftShooterMotor{
    shooterSubsystemConstants::leftShooterMotorID,
    shooterSubsystemConstants::motorPIDConfig,
    shooterSubsystemConstants::motorConversion,
    rmb::noFeedforward<units::radians>,
    {rightShooterMotor}
  };

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
