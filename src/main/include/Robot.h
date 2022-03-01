// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "RobotContainer.h"
#include <frc/TimedRobot.h>

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
  //RobotContainer container;

  // rmb::SparkMaxPositionController<units::radians> turretPositionController {
  //   41,
  //   turretSubsystemConstants::motorPIDConfig,
  //   1/56,
  //   turretSubsystemConstants::motorFeedforward,
  //   {}
  // };

  // rev::CANSparkMax sparkMax {
  //   41, rev::CANSparkMaxLowLevel::MotorType::kBrushless
  // };

  TurretSubsystem turret {};

  frc::Joystick joystick{0};

  units::scalar_t spinDirection = 1.0;
};
