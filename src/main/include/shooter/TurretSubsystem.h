// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <units/velocity.h>
#include <units/angular_velocity.h>

#include <rmb/motorcontrol/sparkmax/SparkMaxVelocityController.h>

#include <Constants.h>

class TurretSubsystem : public frc2::SubsystemBase {
 public:
  TurretSubsystem(units::length::meter_t wheelDiameter = 0_m);

  void spinTo(units::angle::radian_t pos);
  void spinTo(units::length::meter_t pos);

  units::angle::radian_t getAngularPosition();
  units::length::meter_t getLinearPosition();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:

  rmb::SparkMaxPositionController<units::radians> positionController{
    0,
    turretSubsystemConstants::motorPIDConfig,
    1_rad / 1_rad,
    rmb::noFeedforward<units::radians>,
    {}
  };

  const units::length::meter_t wheelDiameter;

  // rmb::SparkMaxPositionController<units::angle::radians> positionController {
  //   turretSubsystemConstants::motorID
  // };
};
