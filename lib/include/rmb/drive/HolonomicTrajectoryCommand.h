#pragma once

#include <frc/Timer.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc/trajectory/Trajectory.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Subsystem.h>

#include "rmb/drive/HolonomicDrive.h"
#include "rmb/drive/DriveOdometry.h"

namespace rmb {
class HolonomicTrajectoryCommand
    : public frc2::CommandHelper<frc2::CommandBase,
                                 HolonomicTrajectoryCommand> {
public:
  HolonomicTrajectoryCommand(const frc::Trajectory &trajectory,
                             HolonomicDrive &drive,
                             const DriveOdometry &odometry,
                             frc::HolonomicDriveController &driveController,
                             std::initializer_list<frc2::Subsystem*> requirements);
  void Initialize();
  void Execute();
  void End(bool interrupted);
  bool IsFinished();

private:
  const frc::Trajectory &trajectory;
  HolonomicDrive &drive;
  const DriveOdometry &odometry;
  frc::HolonomicDriveController &driveController;
  frc::Timer timer;
};
} // namespace rmb