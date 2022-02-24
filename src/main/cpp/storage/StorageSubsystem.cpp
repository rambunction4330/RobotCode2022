// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "storage/StorageSubsystem.h"

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/RunCommand.h>

#include "Constants.h"

StorageSubsystem::StorageSubsystem(const IntakeExtenderSubsystem& extender) : storageWheel(storageConstants::wheelID), colorSensor(frc::I2C::Port::kOnboard), intakeExtender(extender) {
  SetDefaultCommand(frc2::ConditionalCommand( spinStorageCommand(0.5), stopCommand(), [&](){ return !hasBall() && intakeExtender.isExtended(); }));
}

// This method will be called once per scheduler run 
void StorageSubsystem::Periodic() {}

void StorageSubsystem::spinStorage(double speed) {
  storageWheel.Set(speed);
}

std::unique_ptr<frc2::Command> StorageSubsystem::spinStorageCommand(double speed) {
  return std::unique_ptr<frc2::Command>(new frc2::RunCommand([&]() { spinStorage(speed); }, {this}));
}

void StorageSubsystem::stop() {
  storageWheel.Set(0.0);
}

std::unique_ptr<frc2::Command> StorageSubsystem::stopCommand() {
  return std::unique_ptr<frc2::Command>(new frc2::RunCommand([&]() { stop(); }, {this}));
}

bool StorageSubsystem::hasBall() const {
  const static uint32_t threshold = 1000;
  return (const_cast<rev::ColorSensorV3*>(&colorSensor))->GetProximity() > threshold;
}

StorageSubsystem::BallColor StorageSubsystem::ballColor() const {
  if (!hasBall()) {
    return BallColor::NONE;
  }

  const static double redThesh = 100;

  if ((const_cast<rev::ColorSensorV3*>(&colorSensor))->GetColor().red > redThesh) {
    return BallColor::RED;
  } else {
    return BallColor::BLUE;
  }
}