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
 /**
  * Creates a TurretSubsystem with the given wheelDiameter
  * @param wheelDiameter the diameter of the wheel in meters
  */
  TurretSubsystem(units::length::meter_t wheelDiameter = 0_m);

  /**
   * Spin the turret to a position
   * @param pos the position to spin to in radians
   */
  void spinTo(units::angle::radian_t pos);

  /**
   * Spin to turret to a position
   * @param pos the position to spin to in meters
   */
  void spinTo(units::length::meter_t pos);

  /**
   * Spin to an offset of the current position
   * @param pos the offset position in radians
   */
  void spinOffset(units::angle::radian_t pos);

  /**
   * Spin to an offset of the current position
   * @param pos the offset position in meters. I.E angle * circumference
   */
  void spinOffset(units::length::meter_t pos);


  /**
   * Get the position of the turret
   * @return the position of the turret in radians
   */
  units::angle::radian_t getAngularPosition();

  /**
   * Get the position of the turret
   * @return get the linear position of the turret in meters. Ie rotationAngle * circumference
   */
  units::length::meter_t getLinearPosition();

  /**
   * Check if the turret is at the position within the specified error bounds
   * @param pos The target position in radians
   * @return whether or not the turret is at the specified position. True if it is.
   */
  bool isAtPosition(units::angle::radian_t pos);

  /**
   * Check if the turret is at the position within the specified error bounds
   * @param pos The target position in meters
   * @return whether or not the turret is at the specified position. True if it is.
   */
  bool isAtPosition(units::length::meter_t pos);

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
