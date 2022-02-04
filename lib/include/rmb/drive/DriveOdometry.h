#pragma once

#include <frc/geometry/Pose2d.h>

namespace rmb {
/**
 * Odometry for holonomic drive. This class contains functions for managing
 * the robots location on the playing field.
 */
class DriveOdometry {
public:
  /**
   * A pure virtual member that returns the current pose.
   * @see updatePose()
   * @see resetPose()
   * @return The current position of the robot
   */
  virtual const frc::Pose2d &getPose() const = 0;

  /**
   * A pure virtual member that updates the pose.
   * @see getPose()
   * @see resetPose()
   * @return The updated position
   */
  virtual const frc::Pose2d &updatePose() = 0;

  /**
   * A pure virtual member that resets the pose.
   * @see getPose()
   * @see updatePose()
   * @return The pose as a Pose2d
   */
  virtual void resetPose(const frc::Pose2d &pose = frc::Pose2d()) = 0;
};
} // namespace rmb