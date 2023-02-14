#pragma once

#include <iostream>
#include <vector>

#include "MotorUtils.h"

class MotorControl {
 public:
  MotorControl(VehicleType vehicle_type = VehicleType::BLUEROV2, uint8_t num_motors = 6);

  ~MotorControl() = default;

  VehicleType vehicle_type_;

  // Motor configuration
  uint8_t num_motors_;
  std::vector<float> forward_factor_;
  std::vector<float> lateral_factor_;
  std::vector<float> throttle_factor_;
  std::vector<float> yaw_factor_;
  std::vector<float> roll_factor_;
  std::vector<float> pitch_factor_;

  std::vector<float> directions_;

  std::vector<int> motor_pwm_;

  void setMotorDirection(uint8_t motor_num, float direction);
  std::vector<float> thrustToMotorIntensities(const float forward,
                                              const float lateral,
                                              const float throttle,
                                              const float yaw,
                                              const float roll,
                                              const float pitch);

 private:
  void setup();
  void addMotorRaw6Dof(uint8_t motor_num,
                       float forward_factor,
                       float lateral_factor,
                       float throttle_factor,
                       float roll_factor,
                       float pitch_factor,
                       float yaw_factor);
};