#pragma once

#include <iostream>
#include <vector>

#include "MotorUtils.h"

class MotorControl {
 public:
  MotorControl(VehicleType vehicle_type = VehicleType::BLUEROV2, int num_motors = 6);

  ~MotorControl() = default;

  VehicleType vehicle_type_;

  // Motor configuration
  int num_motors_;
  std::vector<double> forward_factor_;
  std::vector<double> lateral_factor_;
  std::vector<double> throttle_factor_;
  std::vector<double> yaw_factor_;
  std::vector<double> roll_factor_;
  std::vector<double> pitch_factor_;

  std::vector<double> directions_;

  std::vector<int> motor_pwm_;

  int max_motor_pwm_;
  int offset_pwm_;

  void setMotorDirection(int motor_num, double direction);
  std::vector<double> thrustToMotorIntensities(const std::vector<double>& thrust_vector);
  std::vector<int> motorIntensitiesToPWM(const std::vector<double>& motor_intensities);
  std::vector<int> getMotorPWM(const std::vector<double>& thrust_vector);

 private:
  void setup();
  void addMotorRaw6Dof(int motor_num,
                       float forward_factor,
                       float lateral_factor,
                       float throttle_factor,
                       float roll_factor,
                       float pitch_factor,
                       float yaw_factor);
};