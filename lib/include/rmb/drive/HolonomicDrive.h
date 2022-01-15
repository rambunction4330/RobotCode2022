#pragma once

#include <units/velocity.h>
#include <units/angular_velocity.h>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/geometry/Translation2d.h>

namespace rmb {
  class HolonomicDrive {
   public:
    virtual void driveChassisSpeeds(frc::ChassisSpeeds chassisSpeeds, 
                                    frc::Translation2d centerofRotation = frc::Translation2d()) = 0;
    
    virtual void driveCartesian(double ySpeed, double xSpeed, double zRotation);
    virtual void drivePolar(double magnitude, double angle, double zRotation);

    virtual units::meters_per_second_t getMaxVel() = 0;
    virtual units::radians_per_second_t getMaxRotVel() = 0;
  };
} // rmb namespace