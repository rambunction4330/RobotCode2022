#include "rmb/command/SuicidalCommand.h"

namespace rmb {
class SuicidalCommand : public frc2::CommandHelper<frc2::CommandBase, SuicidalCommand> {
  public:
    SuicidalCommand(std::unique_ptr<frc2::Command> command) : command(std::move(command)) {}

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

void scheduelSuicidalCommand(std::unique_ptr<frc2::Command> command) {
  frc2::CommandScheduler::GetInstance().Schedule(new SuicidalCommand(std::move(command)));  
}
}