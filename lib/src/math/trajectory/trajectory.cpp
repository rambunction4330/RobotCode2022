#include <rmb/math/trajectory/trajectory.h>

namespace rmb {
namespace trajectory {

Trajectory trajectoryFromEntryAngle(units::meter_t px, units::meter_t py, units::radian_t entryAngle) {
    Trajectory trajectory{};

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

Trajectory trajectoryFromVelocity(units::meter_t px, units::meter_t py, units::meters_per_second_t v) {
    Trajectory trajectory;
    
    // angle
    units::radian_t a = units::math::atan(
        (units::math::pow<2>(v) + units::math::sqrt(units::math::pow<4>(v) - g * (g * units::math::pow<2>(px) + 2 * units::math::pow<2>(v) * py))) / 
        (g * px)
    );

    trajectory.angle = a;
    trajectory.velocity = v;

    return trajectory;
}

Trajectory trajectoryFromAngle(units::meter_t px, units::meter_t py, units::radian_t a) {
    Trajectory trajectory;

    units::velocity::meters_per_second_t v = units::math::sqrt(
        (g * px) /
        (2 * units::math::cos(a) * (units::math::sin(a) - ((py * units::math::cos(a)) / px)))
    );

    trajectory.angle = a;
    trajectory.velocity = v;

    return trajectory;
}

} // namespace trajectory
} // namespace rmb