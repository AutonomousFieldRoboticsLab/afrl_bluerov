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

