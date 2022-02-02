// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"


#include <frc2/command/CommandScheduler.h>
#include <frc/Timer.h>

#include <units/time.h>
#include <units/angular_velocity.h>

void Robot::RobotInit() {

  throttle = this->shuffleBoardTab.Add("Throttle Speed", 0.2f).WithWidget(frc::BuiltInWidgets::kNumberSlider).GetEntry();
  timer.Reset();
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
  smMotorControllerFr.setInverted(true);
  smMotorControllerRR.setInverted(true);
}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  timer.Reset();
  timer.Start();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {

  // const auto JSX = this->joystick.getX();
  // if(JSX)
  // {
  //   this->smMotorControllerFL.setVelocity(0.5_mps);
  // }
  // else
  // {
  //   this->smMotorControllerFL.setVelocity(0_mps);
  // }

  // drive.driveCartesian(-joystick.getY(), -joystick.getX(), joystick.getTwist());
  // odometry.updatePose();
  // wpi::outs() << "(" << std::to_string(odometry.getPose().X().to<double>()) << "m, " << std::to_string(odometry.getPose().Y().to<double>()) << "m)\n";
  // wpi::outs() << "gyro: " << std::to_string(gyro.GetAngle()) << "deg\n";
  wpi::outs() << "time: " << std::to_string(timer.Get().to<double>()) << "s\n";
  if (timer.Get() <= 1_s) {
    drive.driveChassisSpeeds({1_mps, 0_mps, 0_rpm});
  } else {
    drive.driveChassisSpeeds({0_mps, 0_mps, 0_rpm});
  }
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
