#include <rmb/motorcontrol/SparkMax/SparkMaxVelocityController.h>

namespace rmb {

  template <typename U>
  SparkMaxVelocityController<U>::SparkMaxVelocityController(int deviceID) : 
                                                            sparkMax(deviceID, rev::CANSparkMax::MotorType::kBrushless) {
    sparkMaxEncoder = sparkMax.GetEncoder();
    sparkMaxPIDController = sparkMax.GetPIDController();

  }

  template <typename U>
  SparkMaxVelocityController<U>::SparkMaxVelocityController(
                                                            int deviceID, 
                                                            const PIDConfig& config, 
                                                            ConversionUnit_t conversionUnit
                                                            ) : 
                                                            sparkMax(
                                                                    deviceID, 
                                                                    rev::CANSparkMax::MotorType::kBrushless
                                                                    ), conversion(conversionUnit) {
    
    sparkMax.RestoreFactoryDefaults();

    sparkMaxEncoder = sparkMax.GetEncoder();
    sparkMaxPIDController = sparkMax.GetPIDController();

    sparkMaxPIDController.SetP(config.p);
    sparkMaxPIDController.SetI(config.i);
    sparkMaxPIDController.SetD(config.d);
    sparkMaxPIDController.SetFF(config.f);
    sparkMaxPIDController.SetIZone(config.iZone);
    sparkMaxPIDController.SetIMaxAccum(config.iMaxAccumulator);
    sparkMaxPIDController.SetOutputRange(config.minOutput, config.maxOutput);
    
    if(config.usingSmartMotion) {
      sparkMaxPIDController.SetSmartMotionAllowedClosedLoopError(
        RawUnit_t(config.allowedErr / conversion).to<double>()
      );
      sparkMaxPIDController.SetSmartMotionMaxVelocity(
        RawVelocity_t(config.maxVelocity / conversion).to<double>()
      );
      sparkMaxPIDController.SetSmartMotionMaxAccel(
        RawAccel_t(config.maxAccel / conversion).to<double>()
      );
      sparkMaxPIDController.SetSmartMotionAccelStrategy(config.accelStrategy);
      sparkMaxPIDController.SetSmartMotionMinOutputVelocity(RawVelocity_t(config.minVelocity / conversion).to<double>());
    }
}

  template <typename U>
  void SparkMaxVelocityController<U>::setVelocity(Velocity_t velocity) {
    double setPoint = RawVelocity_t(velocity / conversion).to<double>;
    sparkMaxPIDController.SetReference(setPoint, rev::ControlType::kVelocity);
  }

  template <typename U>
  typename SparkMaxVelocityController<U>::Velocity_t SparkMaxVelocityController<U>::getVelocity() {
    return Velocity_t(RawVelocity_t(sparkMaxEncoder.GetVelocity()) * conversion);
  }

} // namespace rmb