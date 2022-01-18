#include "rmb/drive/MecanumDrive.h"

namespace rmb {
  MecanumDrive::MecanumDrive(VelocityController<units::meters>& fl,
                             VelocityController<units::meters>& fr,
                             VelocityController<units::meters>& rl,
                             VelocityController<units::meters>& rr,
                             frc::MecanumDriveKinematics k,
                             units::meters_per_second_t maxVel,
                             units::radians_per_second_t maxRotVel,
                             frc::Gyro& g) :
                             frontLeft(fl), frontRight(fr), rearLeft(rl), rearRight(rr),
                             kinematics(k), maxVelocity(maxVel), maxRotVelocity(maxRotVel),
                             gyro(g), odometry(kinematics, gyro.GetRotation2d()) {}

  void MecanumDrive::driveWheelSpeeds(frc::MecanumDriveWheelSpeeds wheelSpeeds) {
    // Set velocity for each individual wheel
    frontLeft.setVelocity(wheelSpeeds.frontLeft);
    frontRight.setVelocity(wheelSpeeds.frontRight);
    rearLeft.setVelocity(wheelSpeeds.rearLeft);
    rearRight.setVelocity(wheelSpeeds.rearRight);
  }

  void MecanumDrive::driveChassisSpeeds(frc::ChassisSpeeds chassisSpeeds, 
                                        frc::Translation2d centerofRotation) {
    // Get wheel speeds from kinematics
    frc::MecanumDriveWheelSpeeds wheelSpeeds = kinematics.ToWheelSpeeds(chassisSpeeds, centerofRotation);
    driveWheelSpeeds(wheelSpeeds);
  }

  frc::MecanumDriveWheelSpeeds MecanumDrive::getWheelSpeeds() {
    // Return a struct with velocities
    return {
      frontLeft.getVelocity(),
      frontRight.getVelocity(),
      rearLeft.getVelocity(),
      rearRight.getVelocity()
    };
  }

  const frc::Pose2d& MecanumDrive::getPose() {
    return odometry.GetPose();
  }

  const frc::Pose2d& MecanumDrive::updatePose() {
    return odometry.Update(gyro.GetRotation2d(), getWheelSpeeds());
  }

  void MecanumDrive::resetPose() {
    odometry.ResetPosition(getPose(), gyro.GetRotation2d());
  }
}