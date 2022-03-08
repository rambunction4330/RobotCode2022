#pragma once

#include <units/angle.h>
#include <units/base.h>
#include <units/time.h>

#include <rev/CANSparkMax.h>

#include "rmb/motorcontrol/PositionController.h"
#include "rmb/motorcontrol/feedforward/SimpleMotorFeedforward.h" 
#include <limits>

namespace rmb {
/**
 * A wrapper around the SparkMax motorcontroller that allows for the user to set and get the position of the motor
 * accurately through PID functionallity
 */
template <typename DistanceUnit>
class SparkMaxPositionController : PositionController<DistanceUnit> {
public:
  using Distance_t = typename PositionController<DistanceUnit>::Distance_t;

  using VeloctyUnit = typename PositionController<DistanceUnit>::VelocityUnit;
  using Velocity_t = typename PositionController<DistanceUnit>::Velocity_t;

  using AccelerationUnit =
      typename PositionController<DistanceUnit>::AccelerationUnit;
  using Acceleration_t =
      typename PositionController<DistanceUnit>::Acceleration_t;

  /**
   * The distance units that the SparkMax takes in by default is rotations. The conversion here
   * is one rotation to 2pi radians.   
   */
  using RawUnit =
      typename units::unit<std::ratio<2>, units::radians, std::ratio<1>>;
  using RawUnit_t = typename units::unit_t<RawUnit>;

  /**
   * The velocity units that the SparkMaxes use are done in rotations/minute.
   */
  using RawVelocity =
      typename units::compound_unit<RawUnit, units::inverse<units::minutes>>;
  using RawVelocity_t = typename units::unit_t<RawVelocity>;
  
  /**
   * The SparkMax takes in acceleration in rpm/second. 
   */
  using RawAccel =
      typename units::compound_unit<RawVelocity,
                                    units::inverse<units::seconds>>;
  using RawAccel_t = typename units::unit_t<RawAccel>;

  /**
   * The conversion unit from the user defined DistanceUnit to radians. See SparkMaxVelocityController<DistanceUnit>::ConversionUnit.
   */
  using ConversionUnit =
      typename units::compound_unit<DistanceUnit,
                                    units::inverse<units::radians>>;
  using ConversionUnit_t = typename units::unit_t<ConversionUnit>;

  /**
   * Configuration constants for the SparkMax's PID controller.
   */
  struct PIDConfig {
    double p, i, d, f;
    double iZone, iMaxAccumulator;
    double maxOutput, minOutput;

    // SmartMotion config
    bool usingSmartMotion;
    Velocity_t maxVelocity, minVelocity;
    Acceleration_t maxAccel;
    Distance_t allowedErr;
    rev::SparkMaxPIDController::AccelStrategy accelStrategy;
  };

  /**
   * Type definition for the Follower functionality that the SparkMaxes provide. A follower is a motor that Will "follow" the specified motor. 
   * You can specify followers in the constructor.
   */
  struct Follower {
    int id; /**< The ID of the follower motor */
    rev::CANSparkMax::MotorType motorType; /**< The type of motor. Can be kBrushless or kBrushed. */
    bool inverted; /**< Whether or not the follower motor should be inverted. true if inverted. */
  };

  /** \deprecated
   * Creates a SparkMaxPositionController with the specified deviceID
   */
  [[deprecated("Do not use this constructor in competition. This is not guaranteed to work!")]]
  SparkMaxPositionController(int deviceID);

  /**
   * Creates a SparkMaxPositionController with the specified SparkMax ID, PID configuration, and unit configuration
   * @param deviceID The ID of the target SparkMax motorcontroller
   * @param pidConfig configuration constants for the SparkMax PID controller. These constants will help you tune
   *                  your motor such that it runs smooth within the desired bounds.
   * @param conversion The conversion from the user provided DistanceUnits to radians. 
   *                   See SparkMaxPositionController<DistanceUnit>::ConversionUnit
   * 
   * @param feedForward The Feedforward to be passed to the motor. rmb::noFeedforward<DistanceUnit> is the default
   * @param followers list of motors to follow this motor. The parent motor will construct the children with the given configuration
   *                  and will own the followers.
   * @param ticksPerRevolution The number of ticks per rotation of the motor
   * @param motorType the type of motor, Can be kBrushed or kBrushless
   */
  SparkMaxPositionController(int deviceID, const PIDConfig &pidConfig,
                             ConversionUnit_t conversion = ConversionUnit_t(1), 
                             const Feedforward<DistanceUnit>& feedForward = noFeedforward<DistanceUnit>,
                             std::initializer_list<Follower> followers = {}, bool alternateEncoder = false,
                             int ticksPerRevolution = 4096, rev::CANSparkMax::MotorType motorType = rev::CANSparkMax::MotorType::kBrushless);

  /**
   * Sets the position of the motor.
   * @param position The distance from the reference, or "zero" point to set the motor to in user provided Distance_t. \n 
   *                 See SparkMaxPositionController<DistanceUnit>::resetReference(Distance_t position)
   */
  void setPosition(Distance_t position) override;

  /**
   * Gets the position of the motor according to the SparkMax encoder
   * @return the distance from the reference point in the user defined Distance. The reference point can be set with 
   *         rmb::SparkMaxPositionController< DistanceUnit >::resetRefrence(Distance_t position) 	
   */
  Distance_t getPosition() const override;



  /**
   * Get the raw position(in rotations) of the motor. This is for debug purposes or advanced 
   * users who want to bypass all of the code @theVerySharpFlat has written.
   * @return raw position(in rotations) of the motor according to the encoder
   */
  double getRawPosition() {
    return sparkMaxEncoder -> GetPosition();
  }

  /**
   * Gets the velocity of the motor according to the SparkMax encoder
   * @return the velocity of the motor in user specified velocity units
   */
  Velocity_t getVelocity() override;

  /**
   * Toggles motor inversion based on inverted parameter
   * @param inverted Desired motor inversion status. If this is true, the motor will be inverted.
   */
  inline void setInverted(bool inverted) override {
    sparkMax.SetInverted(inverted);
  };

  /**
   * Gets the inversion status of the motor
   * @return Motor inversion status. If this is true, the motor is inverted.
   */
  inline bool getInverted() override { return sparkMax.GetInverted(); };

  /**
   * Resets the reference point to the provided postiion.
   * @param position The desired reference point. This point will be treated as the 
   *                 "0" for the SparkMaxPositionController<DistanceUnit>::getPosition() 
   *                 and SparkMaxPositionController<DistanceUnit>::setPosition(Distance_t position) calls
   */
  void resetRefrence(Distance_t position) override;

  /**
   * Sets the max allowed position of the motor. If the user attempts to move the motor beyond the maximum position, 
   * the motor will be set to the maximum position.
   * @param max The maximum position in user defined DistanceUnits
   */
  void setMaxPosition(Distance_t max) override;

  /**
   * Get the maximum allowed position of the motor
   * @return The maximum postition in user defined distance units
   */
  Distance_t getMaxPosition() override;

  /**
   * Sets the minimum allowed position of the motor. If the user attempts to move the motor beyond the maximum position, 
   * the motor will be set to the minimum position.
   * @param min The minimum position in user defined distance units
   */
  void setMinPosition(Distance_t min) override;

  /**
   * Get the minimum allowed position of the motor
   * @return The minimum postition in user defined distance units
   */
  Distance_t getMinPosition() override;

  /**
   * Check if the motor is at the specified position within the allowed error-bounds
   * specified in the PIDConfig struct passed in the constructor.
   * @param The setpoint
   * @return true if the motor is at the specified position. 
   */
  bool atPosition(Distance_t position);

  /**
   * Spins to an offset of the current position
   * @param position the offset from the current position 
   */
  void spinOffset(Distance_t position);

  /**
   * Check if the position to be set is within the minimum position and maximum position.
   * @param position The theoretical position
   * @return Whether or not the motor can move to the specified position. true if it can move to the position.
   */
  bool canSetPositionTo(Distance_t position);

  /**
   * Check if the motor position can offset to the specified value.
   * @param offset The theoretical offset
   * @return If the motor can spin to the offset. Returns true if it can.  
   */
  bool canSpinOffsetOf(Distance_t offset);

private:
  rev::CANSparkMax sparkMax;
  std::unique_ptr<rev::RelativeEncoder> sparkMaxEncoder;
  rev::SparkMaxPIDController sparkMaxPIDController;

  ConversionUnit_t conversion;

  RawUnit_t minPosition = RawUnit_t(__DBL_MIN__);
  RawUnit_t maxPosition = RawUnit_t(__DBL_MAX__);

  rev::CANSparkMax::ControlType controlType;
  std::vector<std::unique_ptr<rev::CANSparkMax>> followers;

  const Feedforward<DistanceUnit> &feedforward;

  Distance_t reference = Distance_t(0.0);

  Distance_t allowedError = Distance_t(0.0);
};
} // namespace rmb
