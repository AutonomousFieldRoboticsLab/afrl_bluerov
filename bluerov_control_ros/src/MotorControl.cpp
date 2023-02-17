#include "MotorControl.h"

#include "MotorUtils.h"
#include "Utils.h"

MotorControl::MotorControl(VehicleType vehicle_type, int num_motors)
    : vehicle_type_(vehicle_type), num_motors_(num_motors) {
  forward_factor_.resize(num_motors_);
  lateral_factor_.resize(num_motors_);
  throttle_factor_.resize(num_motors_);
  yaw_factor_.resize(num_motors_);
  roll_factor_.resize(num_motors_);
  pitch_factor_.resize(num_motors_);
  directions_.resize(num_motors_);

  setup();

  max_motor_pwm_ = 200;
  offset_pwm_ = 20;
};

void MotorControl::setup() {
  switch (vehicle_type_) {
    case VehicleType::BLUEROV1:
      std::runtime_error("BlueROV1 is not supported");
      break;
    case VehicleType::BLUEROV2:
      addMotorRaw6Dof(Motors::MOTOR_1, -1.0, -1.0, 0.0, 0.0, 0.0, -1.0);
      addMotorRaw6Dof(Motors::MOTOR_2, -1.0, 1.0, 0.0, 0.0, 0.0, 1.0);
      addMotorRaw6Dof(Motors::MOTOR_3, 1.0, -1.0, 0.0, 0.0, 0.0, 1.0);
      addMotorRaw6Dof(Motors::MOTOR_4, 1.0, 1.0, 0.0, 0.0, 0.0, -1.0);
      addMotorRaw6Dof(Motors::MOTOR_5, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0);
      addMotorRaw6Dof(Motors::MOTOR_6, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0);

      break;
    case VehicleType::BLUEROV_HEAVY:
      std::runtime_error("BlueROV Heavy is not supported");
      break;
    default:
      std::runtime_error("Unknown vehicle type");
      break;
  }

  for (int i = 0; i < num_motors_; i++) {
    setMotorDirection(i, 1.0);
  }
}

void MotorControl::addMotorRaw6Dof(int motor_num,
                                   float forward_factor,
                                   float lateral_factor,
                                   float throttle_factor,
                                   float roll_factor,
                                   float pitch_factor,
                                   float yaw_factor) {
  if (motor_num >= 0 && motor_num < MAX_NUM_MOTORS) {
    forward_factor_[motor_num] = forward_factor;
    lateral_factor_[motor_num] = lateral_factor;
    throttle_factor_[motor_num] = throttle_factor;
    roll_factor_[motor_num] = roll_factor;
    pitch_factor_[motor_num] = pitch_factor;
    yaw_factor_[motor_num] = yaw_factor;
  }
}

void MotorControl::setMotorDirection(int motor_num, double direction) {
  if (motor_num >= 0 && motor_num < MAX_NUM_MOTORS) {
    directions_[motor_num] = direction;
  }
}

std::vector<double> MotorControl::thrustToMotorIntensities(
    const std::vector<double>& thrust_vector) {
  // thrust vector is : x, y, z, roll, pitch, yaw
  std::vector<double> motor_intensities(num_motors_, 0.0);

  for (int i = 0; i < num_motors_; ++i) {
    motor_intensities[i] =
        thrust_vector[0] * forward_factor_[i] + thrust_vector[1] * lateral_factor_[i] +
        thrust_vector[2] * throttle_factor_[i] + thrust_vector[3] * roll_factor_[i] +
        thrust_vector[4] * pitch_factor_[i] + thrust_vector[5] * yaw_factor_[i];
  }

  utils::normalizeVector(motor_intensities);

  for (int i = 0; i < num_motors_; ++i) {
    assert(motor_intensities[i] <= 1.0);
    motor_intensities[i] = motor_intensities[i] * directions_[i];
  }

  return motor_intensities;
}

std::vector<int> MotorControl::motorIntensitiesToPWM(const std::vector<double>& motor_intensities) {
  std::vector<int> motor_pwms(num_motors_, 1500);
  int offset = offset_pwm_;
  for (int i = 0; i < num_motors_; ++i) {
    if (motor_intensities[i] < 0) offset = 0 - offset_pwm_;
    motor_pwms[i] = motor_pwms[i] +
                    static_cast<int>(motor_intensities[i] * static_cast<double>(max_motor_pwm_)) +
                    offset;
  }
  return motor_pwms;
}

std::vector<int> MotorControl::getMotorPWM(const std::vector<double>& thrust_vector) {
  std::vector<double> motor_intensities = thrustToMotorIntensities(thrust_vector);
  return motorIntensitiesToPWM(motor_intensities);
}