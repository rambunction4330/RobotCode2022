// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/length.h>
#include <units/velocity.h>

#include <frc2/command/SubsystemBase.h>

#include <rmb/motorcontrol/sparkmax/SparkMaxVelocityController.h>
#include <rmb/motorcontrol/sparkmax/SparkMaxPositionController.h>

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem() : intakeMotorVel(98), intakeMotorPos(73) {}

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void extend();
  void retract();

  void startSpining();
  void stopSpining();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rmb::SparkMaxVelocityController<units::meters> intakeMotorVel;
  rmb::SparkMaxPositionController<units::meters> intakeMotorPos;
};
