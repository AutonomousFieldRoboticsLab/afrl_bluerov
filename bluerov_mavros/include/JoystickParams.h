//
// Created by bjoshi on 3/8/21.
//

#pragma once

namespace BLUE_ROV2 {

class JoystickParams {
 public:
  void parseYaml public : uint8_t axis_roll = 3;
  uint8_t axis_pitch = 4;
  uint8_t axis_yaw = 0;
  uint8_t axis_throttle = 1;

  uint8_t scale_roll = 1.0;
  uint8_t scale_pitch = 1.0;
  uint8_t scale_yaw = 1.0;
  uint8_t scale_throttle = 1.0;

  uint8_t button_arm = 0;
  uint8_t button_disarm = 1;
  uint8_t button_takeoff = 2;
  uint8_t button_land = 3;
  uint8_t button_enable = 4;
};
}  // namespace BLUE_ROV2