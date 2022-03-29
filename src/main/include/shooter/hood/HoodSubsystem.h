// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rmb/motorcontrol/sparkmax/SparkMaxPositionController.h>
#include <Constants.h>

#include <rmb/math/misc.h>

class HoodSubsystem : public frc2::SubsystemBase {
 public:
  HoodSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void setPosition(units::angle::radian_t position);
  units::angle::radian_t getPosition();
  double getRawPosition() {
      return positionController.getRawPosition();
  }

  bool isAtPosition(units::angle::radian_t position) {
      return positionController.atPosition(position);
  }

  void zero() {
      positionController.resetRefrence(0.0_rad);
  }

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rmb::SparkMaxPositionController<units::radians> positionController {
      hoodSubsystemConstants::motorID,
      hoodSubsystemConstants::hoodPIDConfig,
      hoodSubsystemConstants::hoodConversion,
      hoodSubsystemConstants::hoodFeedforward,
      {},
      true,
      4096,
      rev::CANSparkMax::MotorType::kBrushed
  };
};
