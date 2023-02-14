#pragma once

enum BLUEROV_PRIMITIVES {
  TRASECT = 0,       // Move in a straight line
  SQUARE = 1,        // Perform a square
  BOUSTROPHEDON = 2  // Perform a lawnmower pattern
};

enum FEEDBACK {
  NONE = 0,                // No feedback
  POSE = 1,                // Position feedback from
  ATTITUDE = 2,            // Attitude feedback using IMU only
  ATTITUDE_WITH_DEPTH = 3  // Attitude feedback using IMU and z-direction from depth sensor
};

struct SquareConfig {
  float side_length;  // Side length of the square
  float speed;        // Speed of the vehicle
  float depth;        // Depth of the vehicle
  float time;         // Time to complete side of square
};

struct LawnMowerConfig {
  float side_length;  // Side length of the square
  float speed;        // Speed of the vehicle
  float depth;        // Depth of the vehicle
};
