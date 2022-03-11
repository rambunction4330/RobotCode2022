
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "vision/VisionSubsystem.h"
#include <rmb/io/log.h>

void VisionSubsystem::Periodic() {}

bool VisionSubsystem::IsHubInView() const{

  if (networkInstance.IsConnected()) {
    auto table = networkInstance.GetTable("vision");
    auto value = table->GetEntry("isHubVisible");

    return value.GetBoolean(false);
  }

  return 0;
}

// Vector3<float>
// VisionSubsystem::GetHubPosition() { // Rotation to hub, horizontal distance to
//                                     // hub, hub height
//   Vector3<float> ret;

//   if (networkInstance.IsConnected()) {
//     auto table = networkInstance.GetTable("HubData");
//     auto rotationToHub = table->GetEntry("RotationToHub");
//     auto horizontalDistance = table->GetEntry("HorizontalDistanceToHub");
//     auto hubHeight = table->GetEntry("HeightToHub");

//     ret.x = float(rotationToHub.GetDouble(0));
//     ret.y = float(horizontalDistance.GetDouble(0));
//     ret.z = float(hubHeight.GetDouble(0));

//     return ret;
//   }

//   return {};
// }

units::length::meter_t VisionSubsystem::getHubHorizontalPos() const{

  units::length::meter_t horizontalDistance = -1_m;

  if(networkInstance.IsConnected()) {
    auto table = networkInstance.GetTable("vision");
    horizontalDistance = 
      units::length::meter_t((table->GetEntry("distance")).GetDouble(-1));
  }

  return horizontalDistance;
}

units::length::meter_t VisionSubsystem::getHubHeight() const{

  units::length::meter_t verticalHeight = -1_m;

  if(networkInstance.IsConnected()) {
    auto table = networkInstance.GetTable("vision");
    verticalHeight = units::length::meter_t((table->GetEntry("height")).GetDouble(-1));
  }

  return verticalHeight;
}

units::angle::radian_t VisionSubsystem::getAngleToHub() const{
  
  units::angle::radian_t angle;

  if(networkInstance.IsConnected()) {
    auto table = networkInstance.GetTable("vision");
    angle = units::angle::radian_t((table->GetEntry("deltaAngle")).GetDouble(-1));
  }

  return angle;
}