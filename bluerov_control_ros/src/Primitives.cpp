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
  ros::Rate rate(10);
  ros::Time start_time = ros::Time::now();

  ROS_INFO_STREAM("Executing straight line for " << duration << " secs");
  while (ros::ok && (ros::Time::now() - start_time).toSec() <= duration) {
    // Wait for attitude and depth to be initialized for 10 secs
    tf::StampedTransform transform;

    try {
      tf_listener_.lookupTransform("world", "bluerov", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      break;
    }

    double current_depth = transform.getOrigin().getZ();
    tf::Quaternion q = transform.getRotation();

    Eigen::Vector3d current_rpy;
    tf::Matrix3x3(q).getRPY(current_rpy[0], current_rpy[1], current_rpy[2]);

    double forward_speed = speed_;
    double lateral_speed = 0;
    double throttle_speed = (target_depth - current_depth) / duration;

    double roll_speed = utils::angleErrorRadians(0.0, current_rpy[0]) / M_PI;
    double pitch_speed = utils::angleErrorRadians(0.0, current_rpy[1]) / M_PI;
    double yaw_speed = utils::angleErrorRadians(target_yaw, current_rpy[2]) / M_PI;

    ROS_DEBUG_STREAM_THROTTLE(5, "Current depth: " << current_depth);
    ROS_DEBUG_STREAM_THROTTLE(5,
                              "Current RPY: " << current_rpy[0] * 180.0 / M_PI << "\t"
                                              << current_rpy[1] * 180.0 / M_PI << "\t"
                                              << current_rpy[2] * 180.0 / M_PI);
    std::vector<int> motor_pwms = motor_controller_->getMotorPWM(
        {forward_speed, lateral_speed, throttle_speed, roll_speed, pitch_speed, yaw_speed});
    motor_command_callback_(motor_pwms);
    ros::spinOnce();
    rate.sleep();
  }
}

void MotionPrimitive::executeGlobalAttitude(const double target_depth,
                                            const Eigen::Vector3d& target_rpy) {
  ros::Rate rate(10);
  ros::Time start_time = ros::Time::now();

  ROS_INFO_STREAM("Executing global attitude control with depth: "
                  << target_depth << " and target RPY : " << target_rpy.transpose() * 180.0 / M_PI
                  << " deg.");

  while (ros::ok) {
    // Wait for attitude and depth to be initialized for 10 secs
    tf::StampedTransform transform;

    try {
      tf_listener_.lookupTransform("world", "bluerov", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      break;
    }

    double current_depth = transform.getOrigin().getZ();
    tf::Quaternion q = transform.getRotation();

    Eigen::Vector3d current_rpy;
    tf::Matrix3x3(q).getRPY(current_rpy[0], current_rpy[1], current_rpy[2]);

    if (utils::getAngleError(target_rpy, current_rpy).norm() <= 10.0 * M_PI / 180.0 ||
        (ros::Time::now() - start_time).toSec() >= 120.0) {
      ROS_INFO_STREAM("Attitude reached");
      break;
    }

    double forward_speed = 0;
    double lateral_speed = 0;
    double throttle_speed = (target_depth - current_depth);

    Eigen::Vector3d angular_speed = utils::getAngleError(target_rpy, current_rpy) / M_PI;

    ROS_DEBUG_STREAM_THROTTLE(5, "Current depth: " << current_depth);
    ROS_DEBUG_STREAM_THROTTLE(5,
                              "Current RPY: " << current_rpy[0] * 180.0 / M_PI << "\t"
                                              << current_rpy[1] * 180.0 / M_PI << "\t"
                                              << current_rpy[2] * 180.0 / M_PI);
    std::vector<int> motor_pwms = motor_controller_->getMotorPWM({forward_speed,
                                                                  lateral_speed,
                                                                  throttle_speed,
                                                                  angular_speed[0],
                                                                  angular_speed[1],
                                                                  angular_speed[2]});
    motor_command_callback_(motor_pwms);
    ros::spinOnce();
    rate.sleep();
  }
}

Transect::Transect() : MotionPrimitive(), length_(5.0), duration_(5.0) {}

Transect::Transect(float length, float duration)
    : MotionPrimitive(), length_(length), duration_(duration) {}

Transect::Transect(float length, float duration, float speed, FeedbackMethod feedback_method)
    : MotionPrimitive(speed, feedback_method), length_(length), duration_(duration) {}

Transect::~Transect() {}

bool Transect::executeAttitudeFeedback(int num_of_times) {
  // Wait for attitude and depth to be initialized for 10 secs
  tf::StampedTransform transform;

  try {
    tf_listener_.waitForTransform("world", "bluerov", ros::Time(0), ros::Duration(10.0));
    tf_listener_.lookupTransform("world", "bluerov", ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  tf::Vector3 origin = transform.getOrigin();
  tf::Quaternion q = transform.getRotation();

  Eigen::Vector3d initial_rpy;
  tf::Matrix3x3(q).getRPY(initial_rpy[0], initial_rpy[1], initial_rpy[2]);

  // Execute transect
  double initial_depth = origin.getZ();

  ROS_INFO_STREAM("!! Executing Trasect with Atittude Feedback for " << duration_ << " secs!!");
  ROS_INFO_STREAM("Initial Attitude: " << initial_rpy[0] * 180.0 / M_PI << ", "
                                       << initial_rpy[1] * 180.0 / M_PI << ", "
                                       << initial_rpy[2] * 180.0 / M_PI);
  ROS_INFO_STREAM("Initial Depth: " << initial_depth);

  executeStraightLine(duration_, initial_depth, initial_rpy[2]);
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
  // Wait for attitude and depth to be initialized for 10 secs
  tf::StampedTransform transform;

  try {
    tf_listener_.waitForTransform("world", "bluerov", ros::Time(0), ros::Duration(10.0));
    tf_listener_.lookupTransform("world", "bluerov", ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  tf::Vector3 origin = transform.getOrigin();
  tf::Quaternion q = transform.getRotation();

  Eigen::Vector3d initial_rpy;
  tf::Matrix3x3(q).getRPY(initial_rpy[0], initial_rpy[1], initial_rpy[2]);

  // Execute transect
  double initial_depth = origin.getZ();

  ROS_INFO_STREAM("!! Executing Square with Atittude Feedback for " << duration_ << " secs!!");
  ROS_INFO_STREAM("Initial Attitude: " << initial_rpy[0] * 180.0 / M_PI << ", "
                                       << initial_rpy[1] * 180.0 / M_PI << ", "
                                       << initial_rpy[2] * 180.0 / M_PI);
  ROS_INFO_STREAM("Initial Depth: " << initial_depth);

  // Execute square

  executeStraightLine(duration_, initial_depth, initial_rpy[2]);
  executeGlobalAttitude(initial_depth, initial_rpy + Eigen::Vector3d(0.0, 0.0, M_PI_2));
  executeStraightLine(duration_, initial_depth, initial_rpy[2] + M_PI_2);
  executeGlobalAttitude(initial_depth, initial_rpy + Eigen::Vector3d(0.0, 0.0, M_PI));
  executeStraightLine(duration_, initial_depth, initial_rpy[2] + M_PI);
  executeGlobalAttitude(initial_depth, initial_rpy + Eigen::Vector3d(0.0, 0.0, 3.0 * M_PI_2));
  executeStraightLine(duration_, initial_depth, initial_rpy[2] + 3.0 * M_PI_2);
  executeGlobalAttitude(initial_depth, initial_rpy);

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
bool LawnMower::executeAttitudeFeedback(int num_times) {
  // Wait for attitude and depth to be initialized for 10 secs
  tf::StampedTransform transform;

  try {
    tf_listener_.waitForTransform("world", "bluerov", ros::Time(0), ros::Duration(10.0));
    tf_listener_.lookupTransform("world", "bluerov", ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  tf::Vector3 origin = transform.getOrigin();
  tf::Quaternion q = transform.getRotation();

  Eigen::Vector3d initial_rpy;
  tf::Matrix3x3(q).getRPY(initial_rpy[0], initial_rpy[1], initial_rpy[2]);

  // Execute transect
  double initial_depth = origin.getZ();

  ROS_INFO_STREAM("!! Executing Lawnmower with Atittude Feedback ");
  ROS_INFO_STREAM("Initial Attitude: " << initial_rpy[0] * 180.0 / M_PI << ", "
                                       << initial_rpy[1] * 180.0 / M_PI << ", "
                                       << initial_rpy[2] * 180.0 / M_PI);
  ROS_INFO_STREAM("Initial Depth: " << initial_depth);

  // Execute square

  executeStraightLine(long_strip_duration_, initial_depth, initial_rpy[2]);
  executeGlobalAttitude(initial_depth, initial_rpy + Eigen::Vector3d(0.0, 0.0, M_PI_2));
  executeStraightLine(short_strip_duration_, initial_depth, initial_rpy[2] + M_PI_2);
  executeGlobalAttitude(initial_depth, initial_rpy + Eigen::Vector3d(0.0, 0.0, M_PI));
  executeStraightLine(long_strip_duration_, initial_depth, initial_rpy[2] + M_PI);
  executeGlobalAttitude(initial_depth, initial_rpy + Eigen::Vector3d(0.0, 0.0, M_PI_2));
  executeStraightLine(short_strip_duration_, initial_depth, initial_rpy[2] + M_PI_2);
  executeGlobalAttitude(initial_depth, initial_rpy);
  executeStraightLine(long_strip_duration_, initial_depth, initial_rpy[2]);

  return true;
};
