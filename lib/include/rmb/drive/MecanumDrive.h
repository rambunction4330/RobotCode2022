#pragma once

#include <frc/kinematics/MecanumDriveKinematics.h>
#include <frc/kinematics/MecanumDriveOdometry.h>
#include <frc/kinematics/MecanumDriveWheelSpeeds.h>
#include <units/velocity.h>

#include <AHRS.h>

#include "rmb/drive/HolonomicDrive.h"
#include "rmb/motorcontrol/VelocityController.h"

namespace rmb {
class MecanumDrive : public HolonomicDrive {
public:
  /** 
   * Creates a MecanumDrive with wheels, kinematics, and maximum
   * linear and rotational velocity
   * @param frontLeft Velocity controller for the front left wheel
   * @param frontRight Velocity controller for the front right wheel
   * @param rearLeft Velocity controller for the rear left wheel
   * @param rearRight Velocity controller for the rear right wheel
   * @param kinematics The MecanumDriveKinematics for the drive
   * @param maxVelocity The maximum velocity in meters per second
   * @param maxRotVelocity The maximum rotational velocity in radians per second
   */
  MecanumDrive(VelocityController<units::meters> &frontLeft,
               VelocityController<units::meters> &frontRight,
               VelocityController<units::meters> &rearLeft,
               VelocityController<units::meters> &rearRight,
               const frc::MecanumDriveKinematics &kinematics,
               units::meters_per_second_t maxVelocity,
               units::radians_per_second_t maxRotVelocity);

  /**
   * Function to drive using wheel speeds
   * @param wheelSpeeds Wheel speeds to drive using
   * @see getWheelSpeeds()
   * @see driveChassisSpeeds()
   */
  void driveWheelSpeeds(const frc::MecanumDriveWheelSpeeds &wheelSpeeds);

  /**
   * Function that gets the wheel speeds
   * @return The current wheel speeds
   * @see driveWheelSpeeds()
   * @see getChassisSpeeds()
   */
  frc::MecanumDriveWheelSpeeds getWheelSpeeds() const;

  /**
   * Function to drive using chassis speeds
   * @param chassisSpeeds The chassis speeds to drive at
   * @param centerOfRotation The optionally-specified center of rotation, defaults to 0
   * @see driveWheelSpeeds()
   * @see getChassisSpeeds()
   */
  void driveChassisSpeeds(
      const frc::ChassisSpeeds &chassisSpeeds,
      const frc::Translation2d &centerofRotation = frc::Translation2d());

  /**
   * Function that returns the current chassis speeds
   * @return The current chassis speeds
   * @see driveChassisSpeeds()
   * @see getWheelSpeeds()
   */
  frc::ChassisSpeeds getChassisSpeeds() const;

  /**
   * Function that gets the maximum velocity as specified on creation
   * @return The maximum velocity in meters per second
   * @see getMaxRotVel()
   */
  units::meters_per_second_t getMaxVel() const { return maxVelocity; };

  /**
   * Function that gets the maximum rotational velocity as specified on creation
   * @return The maximum velocity in radians per second
   * @see getMaxVel()
   */
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