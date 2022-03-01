// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkMax.h>

#include "IntakeExtenderSubsystem.h"

class IntakeSpinnerSubsystem : public frc2::SubsystemBase {
public:
  IntakeSpinnerSubsystem(const IntakeExtenderSubsystem &extender);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void spin(double speed = 0.5);
  std::unique_ptr<frc2::Command> spinCommand(double speed = 0.5);
  
  void stop();
  std::unique_ptr<frc2::Command> stopCommand();

  bool isSpinning() const;

private:
  rev::CANSparkMax spinner;

  const IntakeExtenderSubsystem &extender;
};