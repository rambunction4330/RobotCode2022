// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "RobotContainer.h"
#include <frc/TimedRobot.h>

#include <shooter/hood/HoodSubsystem.h>

class Robot : public frc::TimedRobot {
public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void TestInit() override;

private:
  //RobotContainer container;
  HoodSubsystem hoodSubsystem{};

  frc::Joystick stick{0};


//  rev::CANSparkMax positionController {
//      42, rev::CANSparkMax::MotorType::kBrushed
//  };

};
