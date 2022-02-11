#include "rmb/drive/HolonomicDrive.h"

#include <algorithm>
#include <units/math.h>

#include <rmb/io/log.h>

namespace rmb {
void HolonomicDrive::driveCartesian(double ySpeed, double xSpeed,
                                    double zRotation) {
  double y = std::clamp(ySpeed, -1.0, 1.0);
  double x = std::clamp(xSpeed, -1.0, 1.0);
  double t = std::clamp(zRotation, -1.0, 1.0);

  frc::ChassisSpeeds speeds = {y * getMaxVel(), x * getMaxVel(),
                               t * getMaxRotVel()};
  driveChassisSpeeds(speeds);
}

void HolonomicDrive::drivePolar(double magnitude, units::radian_t angle,
                                double zRotation) {
  double m = std::clamp(magnitude, -1.0, 1.0);
  double t = std::clamp(zRotation, -1.0, 1.0);

  driveCartesian(m * units::math::sin(angle), m * units::math::cos(angle), t);
}
} // namespace rmb
