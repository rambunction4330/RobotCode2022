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

const units::meters_per_second_squared_t g = 9.80665_mps_sq; /** Universaly agreed upon gravitation constant. 
                                                                 Source: https://en.wikipedia.org/wiki/Gravitational_acceleration*/

/**
 * Structure representing a trajectory
 */
struct Trajectory {
    units::angle::radian_t angle; /** The initial angle of the projectile*/
    units::velocity::meters_per_second_t velocity; /** The initial velocity of the projectile*/
};

/**
 * Calculates the trajectory required based on the location of the target and the desired
 * entry angle Update stuff please
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
 * <h3> Formulas: </h3>
 * \f
 *   v_{y}=\frac{\left(p_{y}+\frac{1}{2}g\left(\frac{p_{x}}{v_{x}}\right)^{2}\right)}{\frac{p_{x}}{v_{x}}}
 * \f
 * 
 * @param px the x position to the target
 * @param py the y position to the target
 * @param entryAngle the desired angle at which the projectile should enter the target
 */
Trajectory trajectoryFromEntryAngle(units::meter_t px, units::meter_t py, units::radian_t entryAngle);

}

}