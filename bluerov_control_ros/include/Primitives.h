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

struct MotionPrimitive {
  float speed_;              // Maximum speed of the vehicle
  FEEDBACK feeback_method_;  // Feedback method to use

  MotionPrimitive() {
    speed_ = 1.0;
    feeback_method_ = FEEDBACK::ATTITUDE_WITH_DEPTH;
  }

  MotionPrimitive(float speed, FEEDBACK feedback_method) {
    speed_ = speed;
    feeback_method_ = feedback_method;
  }
};

struct Trasect : public MotionPrimitive {
  float length_;  // Length of the trasect
  float time_;    // Time to complete trasect
  float depth_;   // Depth of the vehicle

  Trasect() {
    length_ = 5.0;
    time_ = 5.0;
    depth = 100.0;
  }

  Trasect(float length = 5.0, float time = 5.0, float depth = 100.0) {
    length_ = length;
    time_ = time;
    depth_ = depth;
  }

  Trasect(float length = 5.0,
          float time = 5.0,
          float depth = 100.0,
          float speed = 1.0,
          FEEDBACK feedback_method = FEEDBACK::ATTITUDE_WITH_DEPTH) {
    length_ = length;
    time_ = time;
    depth_ = depth;
    speed_ = speed;
    feeback_method_ = feedback_method;
  }
};

struct Square : public MotionPrimitive {
  float length_;  // Side length of the square
  float depth_;   // Depth of the vehicle
  float time_;    // Time to complete side of square

  Square() {
    length_ = 5.0;
    depth_ = 100.0;
    time_ = 5.0;
  }

  Square(float length = 5.0, float depth = 100.0, float time = 5.0) {
    length_ = length;
    depth_ = depth;
    time_ = time;
  }

  Square(float length = 5.0,
         float depth = 100.0,
         float time = 5.0,
         float speed = 1.0,
         FEEDBACK feedback_method = FEEDBACK::ATTITUDE_WITH_DEPTH) {
    length_ = length;
    depth_ = depth;
    time_ = time;
    speed_ = speed;
    feeback_method_ = feedback_method;
  }
};

struct LawnMower : public MotionPrimitive {
  float depth;                  // Depth of the vehicle
  float long_strip_duration_;   // Time to complete side the longer side
  float short_strip_duration_;  // Time to complete side the shorter side
  float strip_length_;          // Length of the longer side
  float strip_width_;           // Length of the shorter side

  LawnMower() {
    depth_ = 100.0;
    long_strip_duration_ = 10.0;
    short_strip_duration_ = 3.0;
    strip_length_ = 10.0;
    strip_width_ = 3.0;
  }

  LawnMower(float depth = 100.0,
            float long_strip_duration = 10.0,
            float short_strip_duration = 3.0,
            float strip_length = 10.0,
            float strip_width = 3.0) {
    depth_ = depth;
    long_strip_duration_ = long_strip_duration;
    short_strip_duration_ = short_strip_duration;
    strip_length_ = strip_length;
    strip_width_ = strip_width;
  }
};
