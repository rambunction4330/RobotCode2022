// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <units/voltage.h>
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
#include <frc/controller/SimpleMotorFeedforward.h>
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
  using kv_unit = units::compound_unit<units::volts, units::inverse<units::meters_per_second>>;
  using ka_unit = units::compound_unit<units::volts, units::inverse<units::meters_per_second_squared>>;

  rmb::SparkMaxVelocityController<units::meters>::PIDConfig smConfig{};
  RobotContainer container;
  Joystick joystick{};
  frc::ShuffleboardTab& shuffleBoardTab  = frc::Shuffleboard::GetTab("RobotData");
  rmb::SparkMaxVelocityController<units::meters> smMotorControllerFL{ 1, smConfig, rmb::SparkMaxVelocityController<units::meters>::ConversionUnit_t(0.4788) };
  frc::SimpleMotorFeedforward<units::meters> motorFF{units::volt_t(0.10973), units::unit_t<kv_unit>(3.1592), units::unit_t<ka_unit>(0.30746)};
  nt::NetworkTableEntry throttle;
};
