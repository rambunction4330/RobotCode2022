// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <rev/ColorSensorV3.h>

#include "intake/IntakeExtenderSubsystem.h"

class StorageSubsystem : public frc2::SubsystemBase {
 public:

  enum BallColor {RED, BLUE, NONE};

  StorageSubsystem(const IntakeExtenderSubsystem& intakeExtender);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void spinStorage(double speed = 0.5);
  std::unique_ptr<frc2::Command> spinStorageCommand(double speed = 0.5);

  void stop();
  std::unique_ptr<frc2::Command> stopCommand();

  bool hasBall() const;
  BallColor ballColor() const;

 private:
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX storageWheel;
  rev::ColorSensorV3 colorSensor;

  const IntakeExtenderSubsystem& intakeExtender;
};