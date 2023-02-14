#pragma once

#include <cmath>

const uint8_t MAX_NUM_MOTORS = 8;

enum VehicleType {
  BLUEROV1 = 0,       // The first version of BlueROV
  BLUEROV2 = 1,       // BlueROV2 with vectored configuration and 5DoF
  BLUEROV_HEAVY = 2,  // BlueROV2 heavy retrofit with vectored configuration and 6DoF
};

// At this stage blurov only supports upto 8 motors
enum Motors {
  MOTOR_1 = 0,
  MOTOR_2 = 1,
  MOTOR_3 = 2,
  MOTOR_4 = 3,
  MOTOR_5 = 4,
  MOTOR_6 = 5,
  MOTOR_7 = 6,
  MOTOR_8 = 7,
};

template <typename T>
T constrainValue(const T value, const T min, const T max) {
  if (std::isnan(value)) return (min + max) / 2;
  return (value < min) ? min : ((value > max) ? max : value);
}

auto constrainFloat = [](const float value, const float min, const float max) {
  return constrainValue<float>(value, min, max);
};

auto constrainInt = [](const int value, const int min, const int max) {
  return constrainValue<int>(value, min, max);
};