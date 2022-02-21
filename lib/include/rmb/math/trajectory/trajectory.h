/** \file trajectory.h
 * This file contains useful trajectory functions to calculate the angle and speed a ball needs to
 * exit a shooter in order to land in the correct position
 */
#include <frc/controller/ArmFeedforward.h>

#include <units/base.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>

#pragma once

namespace rmb {

/**
 * @brief trajectory math
 * 
 * \ingroup trajectory
 */
namespace trajectory {

const units::meters_per_second_squared_t g = 9.80665_mps_sq; /**< Universaly agreed upon gravitation constant. Source: https://en.wikipedia.org/wiki/Gravitational_acceleration */

/**
 * Structure representing a trajectory
 */
struct Trajectory {
    units::angle::radian_t angle; /**< The initial angle of the projectile*/
    units::velocity::meters_per_second_t velocity; /**< The initial velocity of the projectile*/
};

/**
 * Calculates the trajectory required based on the location of the target and the desired
 * entry angle
 * 
 *                                 < py
 *                      \       /  |
 *      _____           |  hub  |  |
 *     /     \         \|  or   |/ |
 *     |Robot|          |target |  
 *     --------------------------  
 *        ------------------^          
 *                          px       
 * 
 * <h3> Formulas Used: </h3>
 * 
 * 
 * Slope of entry 
 * \f[
 *   s_{e}=\tan\left(-a_{e}\right)
 * \f]
 * 
 * 
 * Velocity in the x direction:
 * \f[
 *   v_{x}=\sqrt{\frac{gp_{x}}{2\left(\frac{p_{y}}{p_{x}}-s_{e}\right)}}
 * \f]
 *
 * 
 * Velocity in the y direction:
 * \f[
 *   v_{y}=\frac{\left(p_{y}+\frac{1}{2}g\left(\frac{p_{x}}{v_{x}}\right)^{2}\right)}{\frac{p_{x}}{v_{x}}}
 * \f]
 * 
 * 
 * Required angle:
 * \f[
 *  a=\arctan\left(\frac{v_{y}}{v_{x}}\right)
 * \f]
 * 
 * 
 * Required Velocity:
 * \f[
 *  v=\sqrt{v_{x}^{2}+v_{y}^{2}}
 * \f]
 * 
 * 
 * @param px the x position to the target
 * @param py the y position to the target
 * @param entryAngle the desired angle at which the projectile should enter the target
 * @return The trajectory the projectile needs to follow to meet the provided criteria
 */
Trajectory trajectoryFromEntryAngle(units::meter_t px, units::meter_t py, units::radian_t entryAngle);


/**
 * 
 * Calculates the required trajectory of the projectile based on the given initial velocity
 * and position.
 * 
 * <h3>Formulas used: </h3>
 * 
 * Angle:
 * \f[
 *  a=\arctan\left(\frac{v^{2}+\sqrt{v^{4}-g\left(gp_{x}^{2}+2v^{2}p_{y}\right)}}{gp_{x}}\right)
 * \f]
 * 
 * 
 * @param px the x position to the target
 * @param py the y position to the target
 * @param velocity the initial velocity of the projectile
 * @return The trajectory the projectile needs to follow to meet the provided criteria
 * @see trajectoryFromEntryAngle
 */
Trajectory trajectoryFromVelocity(units::meter_t px, units::meter_t py, units::meters_per_second_t velocity);

/**
 * 
 * Caculates the required trajectory of a projectile based on the x and y position of the target and the
 * intial angle
 * 
 * <h3>Formulas used: </h3>
 * 
 * Velocity:
 * \f[
 *   v=\sqrt{\frac{gp_{x}}{2\cos\left(a\right)\left(\sin\left(a\right)-\frac{\left(p_{y}\cos\left(a\right)\right)}{p_{x}}\right)}}
 * \f]
 * 
 * 
 * @param px the x distance to the target
 * @param py the y distance to the target
 * @param angle the initial angle of the projectile
 * @return The trajectory(angle and velocity) that the projectile must take to reach (px, py)
 * @see trajectoryFromEntryAngle
 */
Trajectory trajectoryFromAngle(units::meter_t px, units::meter_t py, units::radian_t angle);

} // namespace trajectory
} // namespace rmb