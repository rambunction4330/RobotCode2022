#include "rmb/drive/MecanumDrive.h"

#include <rmb/io/log.h>
namespace rmb {
MecanumDrive::MecanumDrive(VelocityController<units::meters> &fl,
                           VelocityController<units::meters> &fr,
                           VelocityController<units::meters> &rl,
                           VelocityController<units::meters> &rr,
                           const frc::MecanumDriveKinematics &k,
                           units::meters_per_second_t maxVel,
                           units::radians_per_second_t maxRotVel)
    : frontLeft(fl), frontRight(fr), rearLeft(rl), rearRight(rr), kinematics(k),
      maxVelocity(maxVel), maxRotVelocity(maxRotVel) {}

void MecanumDrive::driveWheelSpeeds(
    const frc::MecanumDriveWheelSpeeds &wheelSpeeds) {
  // Set velocity for each individual wheel
  frontLeft.setVelocity(wheelSpeeds.frontLeft);
  frontRight.setVelocity(wheelSpeeds.frontRight);
  rearLeft.setVelocity(wheelSpeeds.rearLeft);
  rearRight.setVelocity(wheelSpeeds.rearRight);
}

frc::MecanumDriveWheelSpeeds MecanumDrive::getWheelSpeeds() const {
  // Return a struct with velocities
  return {frontLeft.getVelocity(), frontRight.getVelocity(),
          rearLeft.getVelocity(), rearRight.getVelocity()};
}

void MecanumDrive::driveChassisSpeeds(
    const frc::ChassisSpeeds &chassisSpeeds,
    const frc::Translation2d &centerofRotation) {
  // Get wheel speeds from kinematics
  frc::MecanumDriveWheelSpeeds wheelSpeeds =
      kinematics.ToWheelSpeeds(chassisSpeeds, centerofRotation);
  driveWheelSpeeds(wheelSpeeds);
}

frc::ChassisSpeeds MecanumDrive::getChassisSpeeds() const {
  return kinematics.ToChassisSpeeds(getWheelSpeeds());
}
} // namespace rmb