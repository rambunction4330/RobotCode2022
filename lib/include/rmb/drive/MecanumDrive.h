#pragma once

#include <units/velocity.h>
#include <frc/kinematics/MecanumDriveKinematics.h>
#include <frc/kinematics/MecanumDriveWheelSpeeds.h>
#include <frc/kinematics/MecanumDriveOdometry.h>
#include <AHRS.h>

#include "HolonomicDrive.h"
#include "rmb/motorcontrol/VelocityController.h"

namespace rmb {
  class MecanumDrive : public HolonomicDrive {
    public:
      // Constructor
      MecanumDrive(VelocityController<units::meters>& frontLeft,
                   VelocityController<units::meters>& frontRight,
                   VelocityController<units::meters>& rearLeft,
                   VelocityController<units::meters>& rearRight,
                   frc::MecanumDriveKinematics kinematics,
                   units::meters_per_second_t maxVelocity,
                   units::radians_per_second_t maxRotVelocity,
                   frc::Gyro& gyro);

      // Functions for moving the robot
      void driveWheelSpeeds(frc::MecanumDriveWheelSpeeds wheelSpeeds);
      void driveChassisSpeeds(frc::ChassisSpeeds chassisSpeeds, 
                              frc::Translation2d centerofRotation = frc::Translation2d());
      frc::MecanumDriveWheelSpeeds getWheelSpeeds();
      
      // Functions for getting velocity values
      units::meters_per_second_t getMaxVel() { return maxVelocity; };
      units::radians_per_second_t getMaxRotVel() { return maxRotVelocity; };

      // Functions dealing with odometry
      const frc::Pose2d& getPose();
      const frc::Pose2d& updatePose();
      void resetPose();

    private:
      // Variables for wheels and kinematics (locations)
      VelocityController<units::meters>& frontLeft;
      VelocityController<units::meters>& frontRight;
      VelocityController<units::meters>& rearLeft;
      VelocityController<units::meters>& rearRight;
      frc::MecanumDriveKinematics kinematics;
      frc::Gyro& gyro;
      frc::MecanumDriveOdometry odometry;
      units::meters_per_second_t maxVelocity;
      units::radians_per_second_t maxRotVelocity;
  };
} // rmb namespace