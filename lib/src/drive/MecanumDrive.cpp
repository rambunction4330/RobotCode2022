#include "rmb/drive/MecanumDrive.h"

namespace rmb {
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