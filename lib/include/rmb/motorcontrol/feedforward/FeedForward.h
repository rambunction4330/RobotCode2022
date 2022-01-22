
#include <units/base.h>
#include <units/time.h>
#include <units/voltage.h>

namespace rmb {
template <typename DistanceUnit> class FeedForward {
public:
  using Distance_t = units::unit_t<DistanceUnit>;
  using VelocityUnit =
      units::compound_unit<DistanceUnit, units::inverse<units::seconds>>;
  using Velocity_t = units::unit_t<VelocityUnit>;
  using AccelerationUnit =
      units::compound_unit<VelocityUnit, units::inverse<units::seconds>>;
  using Acceleration_t = units::unit_t<AccelerationUnit>;

  // Define the units for the constants like this:
  //
  // using KsUnit = units::volts;
  // using Ks_t = units::unit_t<KsUnit>;
  // using KvUnit = units::compound_unit<units::volts,
  // units::inverse<Velocity>>; using Kv_t = units::unit_t<KvUnit>; using KaUnit
  // = units::compound_unit<units::volts, units::inverse<Acceleration>>; using
  // Ka_t = units::unit_t<KaUnit>;

  virtual units::volt_t calculate(Velocity_t velocity, Distance_t distance,
                                  Acceleration_t acceleration) = 0;
};
} // namespace rmb