#include "MotorControl.h"

#include "MotorUtils.h"

MotorControl::MotorControl(VehicleType vehicle_type, uint8_t num_motors)
    : vehicle_type_(vehicle_type), num_motors_(num_motors) {
  forward_factor_.resize(num_motors_);
  lateral_factor_.resize(num_motors_);
  throttle_factor_.resize(num_motors_);
  yaw_factor_.resize(num_motors_);
  roll_factor_.resize(num_motors_);
  pitch_factor_.resize(num_motors_);

  setup();
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

  for (uint8_t i = 0; i < num_motors_; i++) {
    setMotorDirection(i, 1.0);
  }
}

void MotorControl::addMotorRaw6Dof(uint8_t motor_num,
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

void MotorControl::setMotorDirection(uint8_t motor_num, float direction) {
  if (motor_num >= 0 && motor_num < MAX_NUM_MOTORS) {
    directions_[motor_num] = direction;
  }
}

std::vector<double> MotorControl::thrustToMotorIntensities(const float forward,
                                                           const float lateral,
                                                           const float throttle,
                                                           const float yaw,
                                                           const float roll,
                                                           const float pitch) {
  return {};
}
