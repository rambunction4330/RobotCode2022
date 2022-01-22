// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>
using namespace units; // we are lazy

template <typename T> class Vector3 {
public:
  T x, y, z;
};

class VisionSubsystem : public frc2::SubsystemBase {
public:
  VisionSubsystem(std::function<radian_t()> fnBaseRotation,
                  std::function<radian_t()> fnShooterAngle)
      : getBaseRotation(fnBaseRotation), getShooterAngle(fnShooterAngle){};

  void Periodic() override;
  bool IsHubInView();
  Vector3<float> GetHubPosition();

private:
  nt::NetworkTableEntry baseRotation, shooterAngle;
  std::function<radian_t()> getBaseRotation, getShooterAngle;
  nt::NetworkTableInstance networkInstance;
};