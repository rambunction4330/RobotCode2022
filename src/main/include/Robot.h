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
#include <rmb/motorcontrol/sparkmax/SparkMaxVelocityController.h>
#include <rmb/motorcontrol/sparkmax/SparkMaxPositionController.h>
#include "RobotContainer.h"
#include <units/length.h>
#include "driverstation/JoystickSubsystem.h"
#include <frc/controller/SimpleMotorFeedforward.h>
#include "Constants.h"
#include <rmb/drive/MecanumDrive.h>
#include <rmb/drive/MecanumEncoderOdometry.h>
#include <AHRS.h>
#include <frc/SPI.h>

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
  JoystickSubsystem joystick{};
  frc::ShuffleboardTab& shuffleBoardTab  = frc::Shuffleboard::GetTab("RobotData");

  rmb::SimpleMotorFeedforward<units::meters> motorFF{units::volt_t(0.10973), units::unit_t<kv_unit>(3.1592), units::unit_t<ka_unit>(0.30746)};
  rmb::SparkMaxVelocityController<units::meters>::Follower follow = {2, rev::CANSparkMax::MotorType::kBrushless, true};
  rmb::SparkMaxVelocityController<units::meters> smMotorControllerFL{ 1, smConfig, rmb::SparkMaxVelocityController<units::meters>::ConversionUnit_t(0.0762_m / 12_rad), motorFF};
  rmb::SparkMaxVelocityController<units::meters> smMotorControllerFr{ 2, smConfig, rmb::SparkMaxVelocityController<units::meters>::ConversionUnit_t(0.0762_m / 12_rad), motorFF};
  rmb::SparkMaxVelocityController<units::meters> smMotorControllerRL{ 3, smConfig, rmb::SparkMaxVelocityController<units::meters>::ConversionUnit_t(0.0508_m / 12_rad), motorFF};
  rmb::SparkMaxVelocityController<units::meters> smMotorControllerRR{ 4, smConfig, rmb::SparkMaxVelocityController<units::meters>::ConversionUnit_t(0.0762_m / 12_rad), motorFF};
  frc::MecanumDriveKinematics kinematics{{0.303_m, -0.299_m}, {0.303_m, 0.299_m}, {-0.303_m, -0.299_m}, {-0.303_m, 0.299_m,}};
  rmb::MecanumDrive drive{smMotorControllerFL, smMotorControllerFr, smMotorControllerRL, smMotorControllerRR, kinematics, 2_mps, 5000_rpm};
  AHRS gyro{frc::SPI::kMXP};
  rmb::MecanumEncoderOdometry odometry{drive, gyro};
  nt::NetworkTableEntry throttle;
  frc::Timer timer;
    rmb::SparkMaxPositionController<units::meters> smPositionController { 
    4,  
    positionControllerConstants::positionCtrlConfig,
    3_in / 12_rad, 
    motorFF,
    {}
  };
};
