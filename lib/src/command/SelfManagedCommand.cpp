#include "rmb/command/SelfManagedCommand.h"

namespace rmb {
class SelfManaged : public frc2::CommandHelper<frc2::CommandBase, SelfManaged> {
  public:
    SelfManaged(std::unique_ptr<frc2::Command> command) : command(std::move(command)) {
      AddRequirements(command->GetRequirements());
    }

    void Initialize() {command->Initialize(); }
    void Execute() { command->Execute(); }

    void End(bool interrupted) { 
      command->End(interrupted);
      delete this;
    }

    bool IsFinished() { return command->IsFinished(); }

  private:
    std::unique_ptr<frc2::Command> command;
};

void scheduelAsSelfManagedCommand(std::unique_ptr<frc2::Command> command) {
  frc2::CommandScheduler::GetInstance().Schedule(new SelfManaged(std::move(command)));  
}
}