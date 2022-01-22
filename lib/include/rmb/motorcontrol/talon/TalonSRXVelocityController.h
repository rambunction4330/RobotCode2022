
#pragma once

#include <units/angle.h>
#include <units/base.h>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>

#include "rmb/motorcontrol/VelocityController.h"

namespace rmb {
template <typename DistanceUnit>
class TalonSRXVelocityController : public VelocityController<DistanceUnit> {
public:
  using Distance_t = typename VelocityController<DistanceUnit>::Distance_t;
  using VelocityUnit = typename VelocityController<DistanceUnit>::VelocityUnit;
  using Velocity_t = typename VelocityController<DistanceUnit>::Velocity_t;
  using AccelerationUnit =
      typename VelocityController<DistanceUnit>::AccelerationUnit;
  using Acceleration_t =
      typename VelocityController<DistanceUnit>::Acceleration_t;

  using RawUnit =
      typename units::unit<std::ratio<2, 4096>, units::radians, std::ratio<1>>;
  using RawUnit_t = typename units::unit_t<RawUnit>;
  using RawVelocity =
      typename units::compound_unit<RawUnit,
                                    units::inverse<units::deciseconds>>;
  using RawVelocity_t = typename units::unit_t<RawVelocity>;

  using ConversionUnit =
      typename units::compound_unit<DistanceUnit,
                                    units::inverse<units::radians>>;
  using ConversionUnit_t = typename units::unit_t<ConversionUnit>;

  struct PIDConfig {
    double p, i, d, f;
    double iZone, iMaxAccumulator;
    Velocity_t allowableError;
    double maxOutput;
  };

  TalonSRXVelocityController(int deviceNumber);
  TalonSRXVelocityController(
      int deviceNumber,
      const ctre::phoenix::motorcontrol::can::TalonSRXConfiguration &config,
      ConversionUnit_t conversion = 1);
  TalonSRXVelocityController(int deviceNumber, const PIDConfig &config,
                             ConversionUnit_t conversion = 1);

  void setVelocity(Velocity_t velocity) override;
  Velocity_t getVelocity() override;

  inline void setInverted(bool inverted) override {
    talonSRX.SetInverted(inverted);
  }
  inline bool getInverted() override { return talonSRX.GetInverted(); }

private:
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX talonSRX;
  ConversionUnit_t conversion;
};
} // namespace rmb