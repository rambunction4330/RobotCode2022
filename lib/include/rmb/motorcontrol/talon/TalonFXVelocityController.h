
#pragma once

#include <units/base.h>
#include <units/angle.h>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>

#include "rmb/motorcontrol/VelocityController.h"

namespace rmb {
  template<typename DistanceUnit>
  class TalonFXVelocityController: public VelocityController<DistanceUnit>, public frc::MotorSafety {
    public:
      using Distance_t =   typename VelocityController<DistanceUnit>::Distance_t;
      using VelocityUnit = typename VelocityController<DistanceUnit>::VelocityUnit;
      using Velocity_t = typename VelocityController<DistanceUnit>::Velocity_t;
      using AccelerationUnit = typename VelocityController<DistanceUnit>::AccelerationUnit;
      using Acceleration_t = typename VelocityController<DistanceUnit>::Acceleration_t;

      using RawUnit = typename units::unit<std::ratio<2, 2048>, units::radians, std::ratio<1>>;
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

      TalonFXVelocityController(int deviceNumber);
      TalonFXVelocityController(int deviceNumber, const ctre::phoenix::motorcontrol::can::TalonFXConfiguration& config, ConversionUnit_t conversion = 1);
      TalonFXVelocityController(int deviceNumber, const PIDConfig& config, ConversionUnit_t conversion = 1);

      void setVelocity(Velocity_t velocity) override;
      Velocity_t getVelocity() override;

      inline void setInverted(bool invetred) override { talonFX.SetInverted(invetred); }
      inline bool getInverted() override { return talonFX.GetInverted(); }
      inline void disable() override { talonFX.Disable(); }	


      inline void Feed() override { talonFX.Feed(); }
      inline void SetExpiration(units::second_t expirationTime) override { talonFX.SetExpiration(expirationTime); }
      inline units::second_t GetExpiration() const override { return talonFX.GetExpiration(); }
      inline bool IsAlive() const override { return talonFX.IsAlive(); }
      inline void SetSafetyEnabled(bool enabled) override { talonFX.SetSafetyEnabled(enabled); }
      inline bool IsSafetyEnabled() const override { return talonFX.IsSafetyEnabled(); }
      inline void Check() override { talonFX.Check(); }
      inline void StopMotor() override { talonFX.StopMotor(); }
      inline std::string GetDescription() override { return talonFX.GetDescription(); }

    private:
      ctre::phoenix::motorcontrol::can::WPI_TalonFX talonFX;
      ConversionUnit_t conversion;
  };
} // rmb namespace