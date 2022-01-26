#pragma once

#include <units/base.h>
#include <units/time.h>
#include <units/voltage.h>

#include <frc/controller/SimpleMotorFeedforward.h>

#include "rmb/motorcontrol/feedforward/Feedforward.h"

namespace rmb {
template <typename DistanceUnit> class SimpleMotorFeedforward : public Feedforward<DistanceUnit> {
public:
  using Distance_t = typename units::unit_t<DistanceUnit>;
  using VelocityUnit = typename Feedforward<DistanceUnit>::VelocityUnit;
  using Velocity_t = typename Feedforward<DistanceUnit>::Velocity_t;
  using AccelerationUnit = typename Feedforward<DistanceUnit>::AccelerationUnit;
  using Acceleration_t = typename Feedforward<DistanceUnit>::Acceleration_t;

  using KsUnit = units::volts;
  using Ks_t = typename units::unit_t<KsUnit>;
  using KvUnit = typename units::compound_unit<units::volts, units::inverse<VelocityUnit>>; 
  using Kv_t = typename units::unit_t<KvUnit>; 
  using KaUnit = typename units::compound_unit<units::volts, units::inverse<AccelerationUnit>>; 
  using Ka_t = typename units::unit_t<KaUnit>;

  SimpleMotorFeedforward(Ks_t ks, Kv_t kv, Ka_t ka) : feedforward(ks, kv, ka) {}

  units::volt_t calculate(Velocity_t velocity, Distance_t distance, Acceleration_t acceleration)  const {
    return feedforward.Calculate(velocity, acceleration);
  }

private:
  frc::SimpleMotorFeedforward<DistanceUnit> feedforward;
};

template<typename DistanceUnit>
const SimpleMotorFeedforward<DistanceUnit> noFeedforward(typename SimpleMotorFeedforward<DistanceUnit>::Ks_t(0.0), 
                                                         typename SimpleMotorFeedforward<DistanceUnit>::Kv_t(0.0), 
                                                         typename SimpleMotorFeedforward<DistanceUnit>::Ka_t(0.0));

} // namespace rmb