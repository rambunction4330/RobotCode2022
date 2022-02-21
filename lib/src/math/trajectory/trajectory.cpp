#include <rmb/math/trajectory/trajectory.h>

namespace rmb {
namespace trajectory {

Trajectory trajectoryFromEntryAngle(units::meter_t px, units::meter_t py, units::radian_t entryAngle) {
    Trajectory trajectory;

    // slope of entry -> units::dimensionless:scalar_t
    auto se = units::math::tan(-1 * entryAngle);

    // velocity x
    units::meters_per_second_t vx = units::math::sqrt((g * px) / (2 * (py / px - se)));

    // velocity y
    units::meters_per_second_t vy = (py + 0.5 * g * units::math::pow<2>(px / vx)) / (px / vx);

    trajectory.angle    = units::math::atan(vy / vx);
    trajectory.velocity = units::math::sqrt(units::math::pow<2>(vx) + units::math::pow<2>(vy));

    return trajectory;
}
}
}