
#pragma once

#include <frc/geometry/Pose2d.h>

namespace rmb {
class HolonomicDriveOdometry {
public:
  virtual const frc::Pose2d &getPose() const = 0;
  virtual const frc::Pose2d &updatePose() = 0;
  virtual void resetPose(const frc::Pose2d &pose) = 0;
};
} // namespace rmb