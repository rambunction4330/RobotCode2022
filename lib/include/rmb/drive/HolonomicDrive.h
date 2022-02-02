#pragma once

#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

namespace rmb {
class HolonomicDrive {
public:
  virtual void driveChassisSpeeds(
      const frc::ChassisSpeeds &chassisSpeeds,
      const frc::Translation2d &centerofRotation = frc::Translation2d()) = 0;

  virtual frc::ChassisSpeeds getChassisSpeeds() const = 0;

  virtual void driveCartesian(double ySpeed, double xSpeed, double zRotation);
  virtual void drivePolar(double magnitude, units::radian_t angle, double zRotation);

  virtual units::meters_per_second_t getMaxVel() const = 0;
  virtual units::radians_per_second_t getMaxRotVel() const = 0;
};
} // namespace rmb
