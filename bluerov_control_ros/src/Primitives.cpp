#include "Primitives.h"

#include <ros/ros.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <cmath>

#include "MotorUtils.h"
#include "Utils.h"

MotionPrimitive::MotionPrimitive() : motor_controller_(nullptr) {
  speed_ = 1.0;
  feeback_method_ = FeedbackMethod::ATTITUDE_WITH_DEPTH;
  is_pose_initialized_ = false;
  is_attitude_initialized_ = false;
  is_depth_initialized_ = false;

  // TODO(bjoshi:) Find better way to do this.
  motor_controller_ = std::make_unique<MotorControl>(VehicleType::BLUEROV2, 6);
  motor_controller_->setMotorDirection(0, -1.0);
  motor_controller_->setMotorDirection(1, -1.0);
  motor_controller_->setMotorDirection(2, -1.0);
}

MotionPrimitive::MotionPrimitive(float speed, FeedbackMethod feedback_method)
    : motor_controller_(nullptr) {
  speed_ = speed;
  feeback_method_ = feedback_method;
  is_pose_initialized_ = false;
  is_attitude_initialized_ = false;
  is_depth_initialized_ = false;

  // TODO(bjoshi:) Find better way to do this.
  motor_controller_ = std::make_unique<MotorControl>(VehicleType::BLUEROV2, 6);
  motor_controller_->setMotorDirection(0, -1.0);
  motor_controller_->setMotorDirection(1, -1.0);
  motor_controller_->setMotorDirection(2, -1.0);
}

void MotionPrimitive::setAttitude(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  tf::Quaternion q(imu_msg->orientation.x,
                   imu_msg->orientation.y,
                   imu_msg->orientation.z,
                   imu_msg->orientation.w);

  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  current_attitude_ = Eigen::Vector3d(roll, pitch, yaw);

  ROS_INFO_STREAM("Attitude RPY: " << roll * 180.0 / M_PI << "\t" << pitch * 180.0 / M_PI << "\t"
                                   << yaw * 180.0 / M_PI);

  if (!is_attitude_initialized_) {
    // initial_attitude_ = current_attitude_;
    is_attitude_initialized_ = true;
  }
}

void MotionPrimitive::setDepth(const double depth) {
  current_depth_ = depth;
  if (!is_depth_initialized_) {
    // initial_depth_ = current_depth_;
    is_depth_initialized_ = true;
  }
}

void MotionPrimitive::setPose(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
  current_position_ = Eigen::Vector3d(
      pose_msg->pose.position.x, pose_msg->pose.position.y, pose_msg->pose.position.z);

  tf::Quaternion q(pose_msg->pose.orientation.x,
                   pose_msg->pose.orientation.y,
                   pose_msg->pose.orientation.z,
                   pose_msg->pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  current_attitude_ = Eigen::Vector3d(roll, pitch, yaw);

  if (!is_pose_initialized_) {
    // initial_position_ = current_position_;
    // initial_attitude_ = current_attitude_;
    is_pose_initialized_ = true;
  }
}

bool MotionPrimitive::execute(int num_of_times) {
  if (feeback_method_ == FeedbackMethod::ATTITUDE ||
      feeback_method_ == FeedbackMethod::ATTITUDE_WITH_DEPTH) {
    return executeAttitudeFeedback(num_of_times);
  } else if (feeback_method_ == FeedbackMethod::POSE) {
    return executePoseFeedback(num_of_times);
  } else {
    return false;
  }
}

void MotionPrimitive::executeStraightLine(const double duration,
                                          const double target_depth,
                                          const double target_yaw) {
  double forward_speed = speed_;
  double lateral_speed = 0;
  double throttle_speed = (target_depth - current_depth_) / duration;

  double roll_speed = utils::angleErrorRadians(0.0, current_attitude_[0]) / M_PI;
  double pitch_speed = utils::angleErrorRadians(0.0, current_attitude_[1]) / M_PI;
  double yaw_speed = utils::angleErrorRadians(target_yaw, current_attitude_[2]) / M_PI;

  std::vector<int> motor_pwms = motor_controller_->getMotorPWM(
      {forward_speed, lateral_speed, throttle_speed, roll_speed, pitch_speed, yaw_speed});
  motor_command_callback_(motor_pwms);
}

void MotionPrimitive::executeGlobalAttitude(const Eigen::Vector3d& target_attitude) {
  // Making sure that the attitude is between -pi and pi
  // TODO(bjoshi): Make sure that this is the correct way to do this
  // This might backfire if the target attitude is close to pi

  double forward_speed = 0;
  double lateral_speed = 0;
  double throttle_speed = 0;
  Eigen::Vector3d angular_speed = utils::getAngleError(target_attitude, current_attitude_) / M_PI;
  std::vector<int> motor_pwms = motor_controller_->getMotorPWM({forward_speed,
                                                                lateral_speed,
                                                                throttle_speed,
                                                                angular_speed[0],
                                                                angular_speed[1],
                                                                angular_speed[2]});
  motor_command_callback_(motor_pwms);
}

Transect::Transect() : MotionPrimitive(), length_(5.0), duration_(5.0) {}

Transect::Transect(float length, float duration)
    : MotionPrimitive(), length_(length), duration_(duration) {}

Transect::Transect(float length, float duration, float speed, FeedbackMethod feedback_method)
    : MotionPrimitive(speed, feedback_method), length_(length), duration_(duration) {}

Transect::~Transect() {}

bool Transect::executeAttitudeFeedback(int num_of_times) {
  ros::Time start_time = ros::Time::now();
  ros::Rate rate(10);

  // Wait for attitude and depth to be initialized for 10 secs
  while (ros::ok() && (!is_attitude_initialized_ || !is_depth_initialized_)) {
    ros::spinOnce();
    rate.sleep();
    if ((ros::Time::now() - start_time).toSec() > 10.0) {
      ROS_ERROR("Attitude or depth not initialized");
      return false;
    }
  }

  // Execute transect
  Eigen::Vector3d initial_attitude = current_attitude_;
  double initial_depth = current_depth_;

  ROS_INFO_STREAM("!! Executing Trasect with Atittude Feedback for " << duration_ << " secs!!");

  executeStraightLine(duration_, initial_depth, initial_attitude[2]);
  return true;
}

bool Transect::executePoseFeedback(int num_times) { return false; };

Square::Square() : MotionPrimitive(), length_(5.0), duration_(5.0) {}

Square::Square(float length, float duration)
    : MotionPrimitive(), length_(length), duration_(duration) {}

Square::Square(float length, float duration, float speed, FeedbackMethod feedback_method)
    : MotionPrimitive(speed, feedback_method), length_(length), duration_(duration) {}

Square::~Square() {}

bool Square::executeAttitudeFeedback(int num_of_times) {
  ros::Time start_time = ros::Time::now();
  ros::Rate rate(10);

  // Wait for attitude and depth to be initialized for 10 secs
  while (ros::ok() && (!is_attitude_initialized_ || !is_depth_initialized_)) {
    ros::spinOnce();
    rate.sleep();
    if ((ros::Time::now() - start_time).toSec() > 10.0) {
      ROS_ERROR("Attitude or depth not initialized");
      return false;
    }
  }

  // Execute square
  Eigen::Vector3d initial_attitude = current_attitude_;
  double initial_depth = current_depth_;

  executeStraightLine(duration_, initial_depth, initial_attitude[2]);
  executeGlobalAttitude(initial_attitude + Eigen::Vector3d(0.0, 0.0, M_PI_2));
  executeStraightLine(duration_, initial_depth, initial_attitude[2] + M_PI_2);
  executeGlobalAttitude(initial_attitude + Eigen::Vector3d(0.0, 0.0, M_PI));
  executeStraightLine(duration_, initial_depth, initial_attitude[2] + M_PI);
  executeGlobalAttitude(initial_attitude + Eigen::Vector3d(0.0, 0.0, 3.0 * M_PI_2));
  executeStraightLine(duration_, initial_depth, initial_attitude[2] + 3.0 * M_PI_2);
  executeGlobalAttitude(initial_attitude);

  return true;
}

bool Square::executePoseFeedback(int num_times) { return false; };

LawnMower::LawnMower()
    : MotionPrimitive(),
      long_strip_duration_(10.0),
      short_strip_duration_(3.0),
      strip_length_(10.0),
      strip_width_(3.0) {}

LawnMower::LawnMower(float long_strip_duration,
                     float short_strip_duration,
                     float strip_length,
                     float strip_width)
    : MotionPrimitive(),
      long_strip_duration_(long_strip_duration),
      short_strip_duration_(short_strip_duration),
      strip_length_(strip_length),
      strip_width_(strip_width) {}

LawnMower::LawnMower(float long_strip_duration,
                     float short_strip_duration,
                     float strip_length,
                     float strip_width,
                     float speed,
                     FeedbackMethod feedback_method)
    : MotionPrimitive(speed, feedback_method),
      long_strip_duration_(long_strip_duration),
      short_strip_duration_(short_strip_duration),
      strip_length_(strip_length),
      strip_width_(strip_width) {}

LawnMower::~LawnMower() {}

bool LawnMower::executePoseFeedback(int num_times) { return false; };
bool LawnMower::executeAttitudeFeedback(int num_times) { return false; };
