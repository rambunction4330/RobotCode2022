
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "vision/VisionSubsystem.h"

void VisionSubsystem::Periodic() {

    static units::radian_t baseRot, shooterAng;
    this->networkInstance = nt::NetworkTableInstance::GetDefault();
    if(this->networkInstance.IsConnected())  {
        baseRot = this->getBaseRotation();
        shooterAng = this->getShooterAngle();
        auto table = this->networkInstance.GetTable("ShooterRobot");
        
        this->baseRotation = table->GetEntry("baseRotation");
        this->baseRotation.SetDouble(baseRot.to<double>());

        this->shooterAngle = table->GetEntry("shooterAngle");
        this->shooterAngle.SetDouble(shooterAng.to<double>());
    }
}


bool VisionSubsystem::IsHubInView() {
    
    if(this->networkInstance.IsConnected())  {
        auto table = this->networkInstance.GetTable("HubData");
        auto value = table->GetEntry("HubInView");
        
        return value.GetBoolean(0);
    }

    return 0;
}

Vector3<float> VisionSubsystem::GetHubPosition() {//Rotation to hub, horoz distance to hub, hub height
    Vector3<float> ret;
    
    if(this->networkInstance.IsConnected())  {
        auto table = this->networkInstance.GetTable("HubData");
        auto rotationToHub = table->GetEntry("RotationToHub");
        auto horozonalDistance = table->GetEntry("horozontalDistanceToHub");
        auto hubHeight = table->GetEntry("HeightToHub");

        ret.x = float(rotationToHub.GetDouble(0));
        ret.y = float(horozonalDistance.GetDouble(0));
        ret.z = float(hubHeight.GetDouble(0));

        return ret;
    }

    return { };
}