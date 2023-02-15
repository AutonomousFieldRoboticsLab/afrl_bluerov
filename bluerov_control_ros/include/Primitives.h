#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Core>
#include <string>

enum PrimitiveType {
  TRASECT = 0,       // Move in a straight line
  SQUARE = 1,        // Perform a square
  BOUSTROPHEDON = 2  // Perform a lawnmower pattern
};

PrimitiveType primitiveTypeFromString(const std::string primitive_type) {
  if (primitive_type == "trasect") {
    return PrimitiveType::TRASECT;
  } else if (primitive_type == "square") {
    return PrimitiveType::SQUARE;
  } else if (primitive_type == "lawnmower") {
    return PrimitiveType::BOUSTROPHEDON;
  } else {
    // ROS_ERROR("Invalid primitive type: %s", primitive_type.c_str());
    return PrimitiveType::SQUARE;
  }
}

enum FeedbackMethod {
  NONE = 0,                // No feedback
  POSE = 1,                // Position feedback from
  ATTITUDE = 2,            // Attitude feedback using IMU only
  ATTITUDE_WITH_DEPTH = 3  // Attitude feedback using IMU and z-direction from depth sensor
};

FeedbackMethod feedbackMethodFromString(const std::string feedback_method) {
  if (feedback_method == "none") {
    return FeedbackMethod::NONE;
  } else if (feedback_method == "pose") {
    return FeedbackMethod::POSE;
  } else if (feedback_method == "attitude") {
    return FeedbackMethod::ATTITUDE;
  } else if (feedback_method == "attitude_with_depth") {
    return FeedbackMethod::ATTITUDE_WITH_DEPTH;
  } else {
    // ROS_ERROR("Invalid feedback method: %s", feedback_method.c_str());
    return FeedbackMethod::ATTITUDE_WITH_DEPTH;
  }
}

class MotionPrimitive {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  float speed_;                    // Maximum speed of the vehicle
  FeedbackMethod feeback_method_;  // Feedback method to use

  MotionPrimitive();
  MotionPrimitive(float speed, FeedbackMethod feedback_method);

  virtual ~MotionPrimitive() = default;
  virtual bool execute() = 0;

  void setAttitude(const sensor_msgs::Imu::ConstPtr& imu_msg);
  void setDepth(const float depth);
  // void setAttitudeAndDepth(const Eigen::Vector3f& attitude, const float depth);
  void setPose(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

  Eigen::Vector3f attitude_;  // Attitude of the vehicle
  float depth_;               // Depth of the vehicle
  Eigen::Vector3f position_;  // Position of the vehicle
};

class Trasect : public MotionPrimitive {
 public:
  Trasect();
  Trasect(float length = 5.0, float duration = 5.0);
  Trasect(float length = 5.0,
          float duration = 5.0,
          float speed = 1.0,
          FeedbackMethod feedback_method = FeedbackMethod::ATTITUDE_WITH_DEPTH);

  virtual ~Trasect();

  // TODO(bjoshi:) Need to change them to private and add getters and setters

  float length_;    // Length of the trasect
  float duration_;  // duration to complete trasect

  bool execute() override;
};

class Square : public MotionPrimitive {
 public:
  Square();

  Square(float length = 5.0, float duration = 5.0);

  Square(float length = 5.0,
         float duration = 5.0,
         float speed = 1.0,
         FeedbackMethod feedback_method = FeedbackMethod::ATTITUDE_WITH_DEPTH);

  virtual ~Square();

  bool execute() override;

  // TODO(bjoshi:) Need to change them to private and add getters and setters

  float length_;    // Side length of the square
  float duration_;  // duration to complete side of square
};

class LawnMower : public MotionPrimitive {
 public:
  LawnMower();

  LawnMower(float long_strip_duration = 10.0,
            float short_strip_duration = 3.0,
            float strip_length = 10.0,
            float strip_width = 3.0);

  LawnMower(float long_strip_duration = 10.0,
            float short_strip_duration = 3.0,
            float strip_length = 10.0,
            float strip_width = 3.0,
            float speed = 1.0,
            FeedbackMethod feedback_method = FeedbackMethod::ATTITUDE_WITH_DEPTH);

  virtual ~LawnMower();

  bool execute() override;

  // TODO(bjoshi:) Need to change them to private and add getters and setters

  float long_strip_duration_;   // duration to complete side the longer side
  float short_strip_duration_;  // duration to complete side the shorter side
  float strip_length_;          // Length of the longer side
  float strip_width_;           // Length of the shorter side
};
