// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "rmb/drive/NavxAccelerometerOdometry.h"

namespace rmb {
NavxAccelerometerOdometry::NavxAccelerometerOdometry(frc::SPI::Port port, const frc::Pose2d& initialPosition) :
                                                     accelerometer(port), refrence(initialPosition), pose(initialPosition) {}

const frc::Pose2d &NavxAccelerometerOdometry::getPose() const { 
  return pose; 
}

const frc::Pose2d &NavxAccelerometerOdometry::updatePose() { 
  return pose;
}

void NavxAccelerometerOdometry::resetPose(const frc::Pose2d &newPose) {
  refrence = newPose;
  pose = newPose;
}
}
