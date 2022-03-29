// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "RobotContainer.h"
#include <frc/TimedRobot.h>

#include <shooter/hood/HoodSubsystem.h>
#include <rmb/motorcontrol/sparkmax/SparkMaxPositionController.h>

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
  void TestInit() override;
  void TestPeriodic() override;

private:

  RobotContainer container;

  //HoodSubsystem hoodSubsystem{};
  //rev::CANSparkMax flywheelA{43, rev::CANSparkMax::MotorType::kBrushless};
  //rev::CANSparkMax flywheelB{44, rev::CANSparkMax::MotorType::kBrushless};


 // TurretSubsystem turret {};

  //frc::Joystick joystick{0};

  units::scalar_t spinDirection = 1.0;

  std::unique_ptr<frc2::Command> trajectoryCommand;


};
