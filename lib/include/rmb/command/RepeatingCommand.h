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
 * Class that will perpetually repeat a command from `Initialize()` to 
 * `End()`. This is helpful for creating an `frc2::SequentialCommand` that 
 * should repeat perpetually unless interupted.
 **/ 
class RepeatingCommand : public frc2::CommandHelper<frc2::CommandBase, RepeatingCommand> {
public:
  /**
   * Wraps and takes ownership of a command that will repeate perpetually 
   * unless interupted.
   * 
   * @param command Command to be wrapped.
   **/
  RepeatingCommand(std::unique_ptr<frc2::Command> command) : command(std::move(command)) {
    AddRequirements(command->GetRequirements());
  }
  
  void Initialize() { command->Initialize(); }

  void Execute() { 
    if (command->IsFinished()) {
      command->End(false);
      command->Initialize();
    }
    command->Execute(); 
  }

  void End(bool interrupted) {}
  bool IsFinished() { return false; }

private:
  std::unique_ptr<frc2::Command> command;
};
} // namespace rmb