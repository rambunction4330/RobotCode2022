
#pragma once

#include <units/base.h>
#include <units/angle.h>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>

#include "rmb/motorcontrol/VelocityController.h"

namespace rmb {
  template<typename DistanceUnit>
  class TalonSRXVelocityController: public VelocityController<DistanceUnit>, public frc::MotorSafety {
    public:
      using Distance_t =   typename VelocityController<DistanceUnit>::Distance_t;
      using VelocityUnit = typename VelocityController<DistanceUnit>::VelocityUnit;
      using Velocity_t = typename VelocityController<DistanceUnit>::Velocity_t;
      using AccelerationUnit = typename VelocityController<DistanceUnit>::AccelerationUnit;
      using Acceleration_t = typename VelocityController<DistanceUnit>::Acceleration_t;

      using RawUnit = typename units::unit<std::ratio<2, 4096>, units::radians, std::ratio<1>>;
      using RawUnit_t =  typename units::unit_t<RawUnit>;
      using RawVelocity = typename units::compound_unit<RawUnit, units::inverse<units::deciseconds>>;
      using RawVelocity_t = typename units::unit_t<RawVelocity>;

      using ConversionUnit = typename units::compound_unit<DistanceUnit, units::inverse<units::radians>>;
      using ConversionUnit_t = typename units::unit_t<ConversionUnit>;

      struct PIDConfig {
        double p, i, d, f;
        double iZone, iMaxAccumulator;
        Velocity_t allowableError;
        double maxOutput;
      };

      TalonSRXVelocityController(int deviceNumber);
      TalonSRXVelocityController(int deviceNumber, const ctre::phoenix::motorcontrol::can::TalonSRXConfiguration& config, ConversionUnit_t conversion = 1);
      TalonSRXVelocityController(int deviceNumber, const PIDConfig& config, ConversionUnit_t conversion = 1);

      void setVelocity(Velocity_t velocity) override;
      Velocity_t getVelocity() override;

      inline void setInverted(bool invetred) override { talonSRX.SetInverted(invetred); }
      inline bool getInverted() override { return talonSRX.GetInverted(); }
      inline void disable() override { talonSRX.Disable(); }
      inline void stopMotor() override { talonSRX.StopMotor(); }


      inline void Feed() override { talonSRX.Feed(); }
      inline void SetExpiration(units::second_t expirationTime) override { talonSRX.SetExpiration(expirationTime); }
      inline units::second_t GetExpiration() const override { return talonSRX.GetExpiration(); }
      inline bool IsAlive() const override { return talonSRX.IsAlive(); }
      inline void SetSafetyEnabled(bool enabled) override { talonSRX.SetSafetyEnabled(enabled); }
      inline bool IsSafetyEnabled() const override { return talonSRX.IsSafetyEnabled(); }
      inline void Check() override { talonSRX.Check(); }
      inline void StopMotor() override { talonSRX.StopMotor(); }
      inline std::string GetDescription() override { return talonSRX.GetDescription(); }

    private:
      ctre::phoenix::motorcontrol::can::WPI_TalonSRX talonSRX;
      ConversionUnit_t conversion;
  };
} // rmb namespace