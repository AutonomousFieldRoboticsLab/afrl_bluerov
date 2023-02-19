#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>
#include <string>

#include "MotorControl.h"

enum PrimitiveType {
  TRANSECT = 0,      // Move in a straight line
  SQUARE = 1,        // Perform a square
  BOUSTROPHEDON = 2  // Perform a lawnmower pattern
};

PrimitiveType primitiveTypeFromString(const std::string primitive_type) {
  if (primitive_type == "transect") {
    return PrimitiveType::TRANSECT;
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
  virtual bool executeAttitudeFeedback(int num_of_runs) = 0;
  virtual bool executePoseFeedback(int num_of_runs) = 0;

  std::unique_ptr<MotorControl> motor_controller_;

  Eigen::Vector3d current_attitude_;  // Attitude of the vehicle
  double current_depth_;              // Depth of the vehicle
  Eigen::Vector3d current_position_;  // Position of the vehicle

  // Eigen::Vector3d initial_attitude_;  // Attitude of the vehicle
  // double initial_depth_;              // Depth of the vehicle
  // Eigen::Vector3d initial_position_;  // Position of the vehicle

  bool is_attitude_initialized_;  // Flag to check if the primitive has been initialized
  bool is_depth_initialized_;     // Flag to check if the primitive has been initialized
  bool is_pose_initialized_;      // Flag to check if the primitive has been initialized

  void setAttitude(const sensor_msgs::Imu::ConstPtr& imu_msg);
  void setDepth(const double depth);
  void setPose(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

  bool execute(int num_of_runs = 1);
  void executeStraightLine(const double duration, const double depth, const double yaw);
  void executeGlobalAttitude(const double depth, const Eigen::Vector3d& target_attitude);

  tf::TransformListener tf_listener_;

  std::function<void(std::vector<int>&)> motor_command_callback_;
  inline void setMotorCommandCallback(const std::function<void(std::vector<int>&)> func) {
    motor_command_callback_ = func;
  }
};

class Transect : public MotionPrimitive {
 public:
  Transect();
  Transect(float length = 5.0, float duration = 5.0);
  Transect(float length = 5.0,
           float duration = 5.0,
           float speed = 1.0,
           FeedbackMethod feedback_method = FeedbackMethod::ATTITUDE_WITH_DEPTH);

  virtual ~Transect();

  // TODO(bjoshi:) Need to change them to private and add getters and setters

  float length_;    // Length of the transect
  float duration_;  // duration to complete transect

  bool executeAttitudeFeedback(int num_of_times = 1) override;
  bool executePoseFeedback(int num_of_times = 1) override;
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

  // TODO(bjoshi:) Need to change them to private and add getters and setters

  float length_;    // Side length of the square
  float duration_;  // duration to complete side of square

  bool executeAttitudeFeedback(int num_of_times = 1) override;
  bool executePoseFeedback(int num_of_times = 1) override;
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

  // TODO(bjoshi:) Need to change them to private and add getters and setters

  float long_strip_duration_;   // duration to complete side the longer side
  float short_strip_duration_;  // duration to complete side the shorter side
  float strip_length_;          // Length of the longer side
  float strip_width_;           // Length of the shorter side

  bool executeAttitudeFeedback(int num_of_times = 1) override;
  bool executePoseFeedback(int num_times = 1) override;
};
