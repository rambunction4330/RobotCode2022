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

#include <shooter/turret/TurretSubsystem.h>
using namespace units; // we are lazy

// template <typename T> class Vector3 {
// public:
//   T x, y, z;
// };

class VisionSubsystem : public frc2::SubsystemBase {
public:
  VisionSubsystem() {
      networkInstance = nt::NetworkTableInstance::GetDefault();
  }

  void Periodic() override;
  bool IsHubInView() const;

  // Vector3<float> GetHubPositioo();

  /**
   * Get the horizontal distance to the hub.
   * @return the horizontal distance to the hub in meters. 
   */
  units::length::meter_t getHubHorizontalPos() const;

  /**
   * Get the vertical distance to the hub
   * @return height in meters
   */
  units::length::meter_t getHubHeight() const;

  /**
   * Get the angle to the hub
   * @return angle in radians to the hub
   */
  units::angle::radian_t getAngleToHub() const;

private:
  nt::NetworkTableEntry baseRotation, shooterAngle;
  nt::NetworkTableInstance networkInstance;
};