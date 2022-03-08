#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  ClimberSubsystem();

  void ExtendArm();
  void RetractArm();
  bool IsExtended();
  void Periodic() override;

 private:
  bool ClimberState = false;
  using TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;
  TalonSRX leftTalon{ 51 };
  TalonSRX rightTalon{ 52 };
};
