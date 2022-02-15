#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Subsystem.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>

namespace rmb {
  /**
   * Scheduels a command that will delete itself after `End()` is called. This 
   * is Achived by wraping the command in an internal `SelfManaged` that 
   * takes ownership of the command. This `SelfManagedCommand` is schedueld and
   * will delete itself and the inital command with it once it is ended.
   * 
   * @param command Command to be schedueled (construt in place).
   **/
  void scheduelAsSelfManagedCommand(std::unique_ptr<frc2::Command> command);
} // namespace rmb