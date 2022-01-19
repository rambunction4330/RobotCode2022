#pragma once

#include <frc/trajectory/Trajectory.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc/Timer.h>

#include "HolonomicDrive.h"


namespace rmb {
  class HolonomicTrajectoryCommand : public frc2::CommandHelper<frc2::CommandBase, HolonomicTrajectoryCommand> {
      public:
        HolonomicTrajectoryCommand(frc::Trajectory trajectory, HolonomicDrive& drive, frc::HolonomicDriveController driveController);
        void Initialize();
        void Execute();
        void End(bool interrupted);
        bool IsFinished();
      private:
        frc::Trajectory trajectory;
        frc::HolonomicDriveController driveController;
        HolonomicDrive& drive;
        frc::Timer timer;
  };
}