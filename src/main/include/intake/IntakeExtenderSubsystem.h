// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/length.h>

#include <frc2/command/Command.h>
#include <frc2/command/SubsystemBase.h>

#include <rmb/motorcontrol/sparkmax/SparkMaxPositionController.h>

class IntakeExtenderSubsystem : public frc2::SubsystemBase {
public:
  IntakeExtenderSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void extend();
  std::unique_ptr<frc2::Command> extendCommand();

  void retract();
  std::unique_ptr<frc2::Command> retractCommand();

  bool isExtended() const;

private:
  rmb::SparkMaxPositionController<units::meters> extender;
};
