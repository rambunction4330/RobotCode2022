// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "intake/IntakeSpinnerSubsystem.h"

#include <frc2/command/Command.h>
#include <frc2/command/RunCommand.h>

#include "Constants.h"

IntakeSpinnerSubsystem::IntakeSpinnerSubsystem(
    const IntakeExtenderSubsystem &extenderSubsystem)
    : spinner(intakeSubsystem::spinnerID,
              rev::CANSparkMax::MotorType::kBrushed),
      extender(extenderSubsystem) {
  SetDefaultCommand(frc2::RunCommand([&]() { stop(); }, {this}));
}

void IntakeSpinnerSubsystem::Periodic() {}

<<<<<<< HEAD
void IntakeSpinnerSubsystem::spin(double speed) {
  if (extender.isExtended()) {
    spinner.Set(speed);
  }
}

=======
void IntakeSpinnerSubsystem::pullIn(double speed) { spinner.Set(speed); }

std::unique_ptr<frc2::Command>
IntakeSpinnerSubsystem::pullInCommand(double speed) {
  return std::unique_ptr<frc2::Command>(
      new frc2::RunCommand([&]() { pullIn(speed); }, {this}));
}


void IntakeSpinnerSubsystem::spitOut(double speed) { spinner.Set(-speed); }

>>>>>>> f9005044a3f992e8ba16294891cda3e625349aae
std::unique_ptr<frc2::Command>
IntakeSpinnerSubsystem::spinCommand(double speed) {
  return std::unique_ptr<frc2::Command>(
      new frc2::RunCommand([&]() { spin(speed); }, {this}));
}

void IntakeSpinnerSubsystem::stop() { spinner.Set(0.0); }

std::unique_ptr<frc2::Command> IntakeSpinnerSubsystem::stopCommand() {
  return std::unique_ptr<frc2::Command>(
      new frc2::RunCommand([&]() { stop(); }, {this}));
}

bool IntakeSpinnerSubsystem::isSpinning() const { return spinner.Get() != 0.0; }