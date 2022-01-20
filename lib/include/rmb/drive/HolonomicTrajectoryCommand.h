
#pragma once

#include <frc/trajectory/Trajectory.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc/Timer.h>

#include "rmb/drive/HolonomicDrive.h"
#include "rmb/drive/HolonomicDriveOdometry.h"


namespace rmb {
  class HolonomicTrajectoryCommand : public frc2::CommandHelper<frc2::CommandBase, HolonomicTrajectoryCommand> {
      public:
        HolonomicTrajectoryCommand(const frc::Trajectory& trajectory, HolonomicDrive& drive, const HolonomicDriveOdometry& odometry, frc::HolonomicDriveController& driveController);
        void Initialize();
        void Execute();
        void End(bool interrupted);
        bool IsFinished();
      private:
        const frc::Trajectory& trajectory;
        HolonomicDrive& drive;
        const HolonomicDriveOdometry& odometry;
        frc::HolonomicDriveController& driveController;
        frc::Timer timer;
  };
}