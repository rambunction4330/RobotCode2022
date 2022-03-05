// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <shooter/ShooterSubsystem.h>
#include <climber/ClimberSubsystem.h>
#include <drivetrain/DriveSubsystem.h>
#include <intake/IntakeSpinnerSubsystem.h>
#include <intake/IntakeExtenderSubsystem.h>
#include <frc2/command/Command.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include "JoystickSubsystem.h"

/**
 * Handles shuffleboard communication and data
 */
class ShuffleBoardSubsystem : public frc2::SubsystemBase {
 public:

  /**
   * Constructs a ShuffleBoardSubsystem
   * @param shooter shootersystem dependency
   * @param joystick shootersystem dependency
   * @param climber shootersystem dependency
   * @param drive shootersystem dependency
   * @param intakeExtender shootersystem dependency
   * @param intakeSpinner shootersystem dependency
   */
  ShuffleBoardSubsystem( const ShooterSubsystem& shooter, const JoystickSubsystem& joystick, const ClimberSubsystem& climber, const DriveSubsystem& drive, const IntakeExtenderSubsystem& intakeExtender, const IntakeSpinnerSubsystem& intakeSpinner) : 
  shooterSubsystem(shooter),
  joystickSubsystem(joystick),
  climberSubsystem(climber),
  driveSubsystem(drive),
  intakeExtenderSubsystem(intakeExtender),
  intakeSpinnerSubsystem(intakeSpinner) {};

  /**
   * Periodic update of networkTables
   */
  void Periodic() override;

  /**
   * Initializes the subsystem networkTables
   */
  void ShuffleBoardInit();
 private:
 const ShooterSubsystem&  shooterSubsystem;
 const JoystickSubsystem& joystickSubsystem;
 const ClimberSubsystem&  climberSubsystem;
 const DriveSubsystem&    driveSubsystem;
 const IntakeExtenderSubsystem&   intakeExtenderSubsystem;
 const IntakeSpinnerSubsystem&   intakeSpinnerSubsystem;
// Network tables vars
 nt::NetworkTableInstance networkInstance;
 frc::ShuffleboardTab&    shuffleBoardTab  = frc::Shuffleboard::GetTab("RobotData");
 nt::NetworkTableEntry    jsThrottle;         // JoystickSubsytem
};
