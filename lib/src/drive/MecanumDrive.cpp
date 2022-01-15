#include "rmb/drive/MecanumDrive.h"

namespace rmb {
  MecanumDrive::MecanumDrive(VelocityController<units::meters>& fl,
                             VelocityController<units::meters>& fr,
                             VelocityController<units::meters>& rl,
                             VelocityController<units::meters>& rr,
                             frc::MecanumDriveKinematics k,
                             units::meters_per_second_t maxVel,
                             units::meters_per_second_t maxRotVel) :
                             frontLeft(fl), frontRight(fr), rearLeft(rl), rearRight(rr),
                             kinematics(k), maxVelocity(maxVel), maxRotVelocity(maxRotVel) {}

  void MecanumDrive::driveWheelSpeeds(frc::MecanumDriveWheelSpeeds wheelSpeeds) {
    // Set velocity for each individual wheel
    frontLeft.setVelocity(wheelSpeeds.frontLeft);
    frontRight.setVelocity(wheelSpeeds.frontRight);
    rearLeft.setVelocity(wheelSpeeds.rearLeft);
    rearRight.setVelocity(wheelSpeeds.rearRight);
  }

  void MecanumDrive::driveChassisSpeeds(frc::ChassisSpeeds chassisSpeeds, 
                                        frc::Translation2d centerofRotation = frc::Translation2d()) {
    // Get wheel speeds from kinematics
    frc::MecanumDriveWheelSpeeds wheelSpeeds = kinematics.ToWheelSpeeds(chassisSpeeds, centerofRotation);
    driveWheelSpeeds(wheelSpeeds);
  }
}