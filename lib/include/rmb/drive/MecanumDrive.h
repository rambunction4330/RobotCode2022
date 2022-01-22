#pragma once

#include <AHRS.h>
#include <frc/kinematics/MecanumDriveKinematics.h>
#include <frc/kinematics/MecanumDriveOdometry.h>
#include <frc/kinematics/MecanumDriveWheelSpeeds.h>
#include <units/velocity.h>

#include "rmb/drive/HolonomicDrive.h"
#include "rmb/motorcontrol/VelocityController.h"

namespace rmb {
class MecanumDrive : public HolonomicDrive {
public:
  // Constructor
  MecanumDrive(VelocityController<units::meters> &frontLeft,
               VelocityController<units::meters> &frontRight,
               VelocityController<units::meters> &rearLeft,
               VelocityController<units::meters> &rearRight,
               const frc::MecanumDriveKinematics &kinematics,
               units::meters_per_second_t maxVelocity,
               units::radians_per_second_t maxRotVelocity);

  // Functions for moving the robot
  void driveWheelSpeeds(const frc::MecanumDriveWheelSpeeds &wheelSpeeds);

  frc::MecanumDriveWheelSpeeds getWheelSpeeds() const;

  void driveChassisSpeeds(
      const frc::ChassisSpeeds &chassisSpeeds,
      const frc::Translation2d &centerofRotation = frc::Translation2d());

  frc::ChassisSpeeds getChassisSpeeds() const;

  // Functions for getting velocity values
  units::meters_per_second_t getMaxVel() const { return maxVelocity; };
  units::radians_per_second_t getMaxRotVel() const { return maxRotVelocity; };

private:
  // Variables for wheels and kinematics (locations)
  VelocityController<units::meters> &frontLeft;
  VelocityController<units::meters> &frontRight;
  VelocityController<units::meters> &rearLeft;
  VelocityController<units::meters> &rearRight;
  const frc::MecanumDriveKinematics &kinematics;
  units::meters_per_second_t maxVelocity;
  units::radians_per_second_t maxRotVelocity;

  friend class MecanumEncoderOdometry;
};
} // namespace rmb