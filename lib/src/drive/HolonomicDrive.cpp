#include "rmb/drive/HolonomicDrive.h"

#include <algorithm>

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

void HolonomicDrive::drivePolar(double magnitude, double angle,
                                double zRotation) {
  double m = std::clamp(magnitude, -1.0, 1.0);
  double t = std::clamp(zRotation, -1.0, 1.0);

  double radAngle = units::convert<units::degree, units::radian>(angle);
  driveCartesian(m * sin(radAngle), m * cos(radAngle), t);
}
} // namespace rmb
