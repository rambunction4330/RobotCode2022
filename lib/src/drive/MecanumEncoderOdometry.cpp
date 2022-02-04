#include "rmb/drive/MecanumEncoderOdometry.h"

namespace rmb {
MecanumEncoderOdometry::MecanumEncoderOdometry(MecanumDrive &d,
                                               const frc::Gyro &g,
                                               const frc::Pose2d &pose)
    : drive(d), gyro(g),
      odometry(drive.kinematics, gyro.GetRotation2d(), pose) {}

const frc::Pose2d &MecanumEncoderOdometry::getPose() const {
  return odometry.GetPose();
}

const frc::Pose2d &MecanumEncoderOdometry::updatePose() {
  return odometry.Update(gyro.GetRotation2d(), drive.getWheelSpeeds());
}

void MecanumEncoderOdometry::resetPose(const frc::Pose2d &pose) {
  odometry.ResetPosition(pose, gyro.GetRotation2d());
}

} // namespace rmb