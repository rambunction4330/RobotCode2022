// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/Timer.h>
#include <frc2/command/CommandScheduler.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/time.h>

#include <rmb/io/log.h>

#include <rmb/drive/HolonomicTrajectoryCommand.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include <rmb/command/SelfManagedCommand.h>

void Robot::RobotInit() { 
  //turretPositionController.resetRefrence(0_rad);
  // container.shuffleBoard.ShuffleBoardInit();
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
  // container.shuffleBoard.Periodic();
  }

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
  //container.getTeleopDriveCommand().Cancel();
}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  container.resetOdometry();
  container.getTeleopDriveCommand().Cancel();
  //rmb::scheduelAsSelfManagedCommand(container.getTrajectoryCommand());
  trajectoryCommand = container.getTrajectoryCommand();
  trajectoryCommand->Schedule();
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
//   frc2::CommandScheduler::GetInstance().Schedule(
//       &container.getTeleopDriveCommand());

   container.getTeleopDriveCommand().Schedule();
   container.getManualShooterCommand()->Schedule();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**s
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();
}

void Robot::TestInit() {

}



#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
