#pragma once

#include <frc/interfaces/Gyro.h>

#include "rmb/drive/HolonomicDrive.h"
#include "rmb/drive/HolonomicDriveOdometry.h"
#include "rmb/drive/MecanumDrive.h"

namespace rmb {
class MecanumEncoderOdometry : public HolonomicDriveOdometry {
public:
  MecanumEncoderOdometry(MecanumDrive &drive, const frc::Gyro &gyro,
                         const frc::Pose2d &initalPose = frc::Pose2d());

  const frc::Pose2d &getPose() const;
  const frc::Pose2d &updatePose();
  void resetPose(const frc::Pose2d &pose);

private:
  MecanumDrive &drive;
  const frc::Gyro &gyro;
  frc::MecanumDriveOdometry odometry;
};
} // namespace rmb