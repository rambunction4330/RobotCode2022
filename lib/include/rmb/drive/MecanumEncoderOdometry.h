#pragma once

#include <frc/interfaces/Gyro.h>

#include "rmb/drive/DriveOdometry.h"
#include "rmb/drive/HolonomicDrive.h"
#include "rmb/drive/MecanumDrive.h"

namespace rmb {
class MecanumEncoderOdometry : public DriveOdometry {
public:
  /** 
   * Creates a MecanumEncoderOdemetry with the mecanum drive,
   * gyro, and initial pose (location)
   * @param drive Mecanum drive of the robot
   * @param gyro The gyroscope
   * @param initialPose The initial position of the robot
   */
  MecanumEncoderOdometry(MecanumDrive &drive, const frc::Gyro &gyro,
                         const frc::Pose2d &initalPose = frc::Pose2d());

  /**
   * Getter to return the current pose
   * @return The current position
   * @see updatePose()
   * @see resetPose()
   */
  const frc::Pose2d &getPose() const;

  /**
   * Function to update the pose
   * @return The update position
   * @see getPose()
   * @see resetPose()
   */
  const frc::Pose2d &updatePose();

  /**
   * Function to reset the pose
   * @see getPose()
   * @see updatePose()
   */
  void resetPose(const frc::Pose2d &pose);

private:
  MecanumDrive &drive;
  const frc::Gyro &gyro;
  frc::MecanumDriveOdometry odometry;
};
} // namespace rmb