// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "intake/IntakeExtenderSubsystem.h"

#include <frc2/command/RunCommand.h>

#include "Constants.h"

#include <rmb/io/log.h>

IntakeExtenderSubsystem::IntakeExtenderSubsystem()
    : extender(intakeSubsystem::extenderID, intakeSubsystem::extenderPIDConfig,
               intakeSubsystem::extenderConvertion,
               intakeSubsystem::extenderFeedforward,
               {intakeSubsystem::extenderFollower}) {
<<<<<<< HEAD
  extender.resetRefrence(0.0_m);
  SetDefaultCommand(frc2::RunCommand([this]() { retract(); }, {this}));
=======
  extender.setMinPosition(0.0_m);
  extender.setMaxPosition(0.25_m);
>>>>>>> f9005044a3f992e8ba16294891cda3e625349aae
}

// This method will be called once per scheduler run
void IntakeExtenderSubsystem::Periodic() {
<<<<<<< HEAD
  wpi::outs() << std::to_string(extender.getPosition().to<double>()) << wpi::endl;
=======
  wpi::outs() << extender.getPosition() << wpi::endl;
>>>>>>> f9005044a3f992e8ba16294891cda3e625349aae
}

void IntakeExtenderSubsystem::extend() {
  extender.setPosition(intakeSubsystem::extenderOut);
}

std::unique_ptr<frc2::Command> IntakeExtenderSubsystem::extendCommand() {
  return std::unique_ptr<frc2::Command>(
      new frc2::RunCommand([this]() { extend(); }, {this}));
}

void IntakeExtenderSubsystem::retract() {
  extender.setPosition(intakeSubsystem::extenderIn);
}

std::unique_ptr<frc2::Command> IntakeExtenderSubsystem::retractCommand() {
  return std::unique_ptr<frc2::Command>(
      new frc2::RunCommand([this]() { retract(); }, {this}));
}

bool IntakeExtenderSubsystem::isExtended() const {
  // TODO: Needs to be implemented
 return (const_cast<rmb::SparkMaxPositionController<units::meters>*>(&extender))->atPosition(intakeSubsystem::extenderOut);
}