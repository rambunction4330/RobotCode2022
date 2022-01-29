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
  IntakeSubsystem() : intakeMotorVel(-1), intakeMotorPos(-1) {}

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void deployIntake() { //moves intake out from starting pos 
    intakeMotorPos.setPosition(100_m);
  }

  void disengageIntake() { //moves intake to starting pos
    intakeMotorPos.setPosition(0_m);
  }

  void stopSpinning() { //stops spinning discs to collect balls
    intakeMotorVel.setVelocity(0_mps);
  }

  void startSpinning() { //starts spinning discs to collect balls
    intakeMotorVel.setVelocity(100_mps);
  }

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rmb::SparkMaxVelocityController<units::meters> intakeMotorVel;
  rmb::SparkMaxPositionController<units::meters> intakeMotorPos;
};
