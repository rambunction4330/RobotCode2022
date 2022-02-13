#pragma once

#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

namespace rmb {
/**
 * The Holonomic Drive class. Manages movement
 * and the retrieval of certain values such as
 * velocity from the robot.
 */
class HolonomicDrive {
public:
  /**
   * A pure virtual member that drives using chassis
   * speeds.
   * @param chassisSpeeds The chassis speeds to drive
   * at.
   * @param centerOfRotation Translation2d that provides
   * the center pivot point for the robot.
   * @see getChassisSpeeds()
   * @see driveCartesian()
   * @see drivePolar()
   */
  virtual void driveChassisSpeeds(
      const frc::ChassisSpeeds &chassisSpeeds,
      const frc::Translation2d &centerofRotation = frc::Translation2d()) = 0;
  
  /**
   * A pure virtual member that returns the current
   * chassis speeds.
   * @see driveChassisSpeeds()
   * @see getMaxVel()
   * @see getMaxRotVel()
   */
  virtual frc::ChassisSpeeds getChassisSpeeds() const = 0;

  /**
   * A pure virtual member that drives using
   * cartesian coordinates.
   * @param ySpeed Double that gives the speed
   * in the y-direction.
   * @param xSpeed Double that gives the speed
   * in the x-direction.
   * @param zRotation Double that gives the rotation
   * along the z-axis.
   * @see driveChassisSpeeds()
   * @see drivePolar()
   */
  virtual void driveCartesian(double ySpeed, double xSpeed, double zRotation);
  
  /**
   * A pure virtual member that drives using
   * cartesian coordinates.
   * @param magnitude Double that gives the magnitude
   * in the direction specified by the angle parameter.
   * @param angle Double that gives the angle for the 
   * direction to travel in.
   * @param zRotation Double that gives the rotation
   * along the z-axis.
   * @see driveChassisSpeeds()
   * @see driveCartesian()
   */
  virtual void drivePolar(double magnitude, units::radian_t angle,
                          double zRotation);

  /**
   * A pure virtual member that returns
   * the maximum linear velocity as specified.
   * @return The maximum velocity in
   * meters per second
   */
  virtual units::meters_per_second_t getMaxVel() const = 0;

  /**
   * A pure virtual member that returns
   * the maximum rotational velocity as specified.
   * @return The maximum rotational velocity in
   * radians per second
   */
  virtual units::radians_per_second_t getMaxRotVel() const = 0;
};
} // namespace rmb
