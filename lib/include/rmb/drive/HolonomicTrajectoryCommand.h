#pragma once

#include <frc/Timer.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc/trajectory/Trajectory.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Subsystem.h>

#include "rmb/drive/DriveOdometry.h"
#include "rmb/drive/HolonomicDrive.h"

namespace rmb {
/**
 * The Holonomic Trajectory Command class.
 * Calculates trajectory and movement for
 * the drive subsystem.
 */
class HolonomicTrajectoryCommand
    : public frc2::CommandHelper<frc2::CommandBase,
                                 HolonomicTrajectoryCommand> {
public:
  HolonomicTrajectoryCommand(
      frc::Trajectory  trajectory, HolonomicDrive &drive,
      const DriveOdometry &odometry,
      frc::HolonomicDriveController &driveController,
      std::initializer_list<frc2::Subsystem *> requirements);
  /**
   * Function to be run on initialization of the command.
   * @see Execute()
   * @see End()
   * @see IsFinished()
   */
  void Initialize();

  /**
   * Function called repeatedly while the command is scheduled.
   * @see Initialize()
   * @see End()
   * @see IsFinished()
   */
  void Execute();

  /**
   * Function called when the command ends or is interrupted.
   * @param interrupted false if ended cleanly or true if interrupted
   * explicitly canceled
   * @see Initialize()
   * @see Execute()
   * @see IsFinished()
   */
  void End(bool interrupted);

  /** Function called repeatedly while the command is scheduled and calls End()
   * when returns true.
   * @see Initialize()
   * @see Execute()
   * @see End()
   */
  bool IsFinished() override;

private:
  const frc::Trajectory trajectory;
  HolonomicDrive &drive;
  const DriveOdometry &odometry;
  frc::HolonomicDriveController &driveController;
  frc::Timer timer;
};
} // namespace rmb