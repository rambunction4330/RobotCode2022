// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <rmb/motorcontrol/SparkMax/SparkMaxVelocityController.h>
#include "RobotContainer.h"
#include <units/length.h>
#include "driverstation/Joystick.h"
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

 private:

  rmb::SparkMaxVelocityController<units::radians>::PIDConfig smConfig{};
  RobotContainer container;
  Joystick joystick{};
  frc::ShuffleboardTab& shuffleBoardTab  = frc::Shuffleboard::GetTab("RobotData");
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX pMotorController{2};
  rmb::SparkMaxVelocityController<units::radians> smMotorController{ 42, smConfig };
  nt::NetworkTableEntry throttle;
};
