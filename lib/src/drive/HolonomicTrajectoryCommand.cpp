#include "rmb/drive/HolonomicTrajectoryCommand.h"

#include <utility>
#include "rmb/io/log.h"
#include <units/math.h>

namespace rmb {
HolonomicTrajectoryCommand::HolonomicTrajectoryCommand(
    frc::Trajectory  t, HolonomicDrive &d,
    const DriveOdometry &o, frc::HolonomicDriveController &dc,
    std::initializer_list<frc2::Subsystem*> requirements)
    : trajectory(std::move(t)), drive(d), odometry(o), driveController(dc) {
      AddRequirements(requirements);
    }

void HolonomicTrajectoryCommand::Initialize() {
  timer.Reset();
  timer.Start();
}

void HolonomicTrajectoryCommand::Execute() {
  units::time::second_t currentTime = timer.Get();
  frc::Trajectory::State desiredState = trajectory.Sample(currentTime);
  auto targetChassisSpeeds =
      driveController.Calculate(odometry.getPose(), desiredState,
                                trajectory.States().back().pose.Rotation());
  drive.driveChassisSpeeds(targetChassisSpeeds);
}

void HolonomicTrajectoryCommand::End(bool interrupted) {
    timer.Stop();
}

bool HolonomicTrajectoryCommand::IsFinished() {
  return timer.HasElapsed(trajectory.TotalTime());
}
} // namespace rmb