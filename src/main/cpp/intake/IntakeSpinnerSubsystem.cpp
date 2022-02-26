// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "intake/IntakeSpinnerSubsystem.h"

#include <frc2/command/Command.h>
#include <frc2/command/RunCommand.h>

#include "Constants.h"

IntakeSpinnerSubsystem::IntakeSpinnerSubsystem()
    : spinner(intakeSubsystem::spinnerID,
              rev::CANSparkMax::MotorType::kBrushed) {}

void IntakeSpinnerSubsystem::Periodic() {}

void IntakeSpinnerSubsystem::pullIn(double speed) { spinner.Set(speed); }

std::unique_ptr<frc2::Command>
IntakeSpinnerSubsystem::pullInCommand(double speed) {
  return std::unique_ptr<frc2::Command>(
      new frc2::RunCommand([&]() { pullIn(speed); }, {this}));
}


void IntakeSpinnerSubsystem::spitOut(double speed) { spinner.Set(-speed); }

std::unique_ptr<frc2::Command>
IntakeSpinnerSubsystem::spitOutCommand(double speed) {
  return std::unique_ptr<frc2::Command>(
      new frc2::RunCommand([&]() { spitOut(speed); }, {this}));
}

void IntakeSpinnerSubsystem::stop() { spinner.Set(0.0); }

std::unique_ptr<frc2::Command> IntakeSpinnerSubsystem::stopCommand() {
  return std::unique_ptr<frc2::Command>(
      new frc2::RunCommand([&]() { stop(); }, {this}));
}