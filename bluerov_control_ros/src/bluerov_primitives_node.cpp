#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>

#include "Primitives.h"

void pressureCallback(const sensor_msgs::FluidPressure::ConstPtr& msg,
                      std::function<void(const double)> depth_callback,
                      double fluid_density) {
  double depth = (msg->fluid_pressure - 101300.0f) / (fluid_density * 9.80665f);
  ROS_INFO_STREAM_THROTTLE(10, "Pressure: " << msg->fluid_pressure << "Depth: " << depth);

  depth_callback(depth);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "bluerov_primitives");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::unique_ptr<MotionPrimitive> motion_primitive_ = nullptr;

  std::string pattern;
  std::string feedback_method_str;

  std::string environment;

  float fluid_density;
  float speed;
  int num_of_runs;

  nh_private.param<std::string>("pattern", pattern, "square");
  nh_private.param<std::string>("feedback_type", feedback_method_str, "attitude_with_depth");
  nh_private.param<float>("speed", speed, 1.0);
  nh_private.param<int>("num_of_runs", num_of_runs, 1);
  nh_private.param<std::string>("environment", environment, "sea");

  PrimitiveType primitive_type = primitiveTypeFromString(pattern);
  FeedbackMethod feedback_method = feedbackMethodFromString(feedback_method_str);

  if (environment == "sea") {
    fluid_density = 1029.0;
  } else if (environment == "air") {
    fluid_density = 1.225;
  } else if (environment == "fresh_water") {
    fluid_density = 997.0;
  } else {
    ROS_ERROR("Invalid environment: %s", environment.c_str());
    ROS_ERROR("Valid environments are: sea, air, fresh_water");
    ROS_ERROR("Defaulting to sea");
    fluid_density = 1029.0;
  }

  ros::Subscriber pose_sub;

  if (primitive_type == PrimitiveType::TRANSECT) {
    float transect_length, duration;
    // if we have acess to VIO Pose, we can execute the exact transect length
    if (feedback_method == FeedbackMethod::POSE) {
      nh_private.param<float>("transect/transect_length", transect_length, 5.0);

    } else {
      nh_private.param<float>("transect/duration", duration, 5.0);
    }
    motion_primitive_ =
        std::make_unique<Transect>(transect_length, duration, speed, feedback_method);
  } else if (primitive_type == PrimitiveType::SQUARE) {
    // if we have acess to VIO Pose, we can execute the exact transect length
    float square_length, duration;
    if (feedback_method == FeedbackMethod::POSE) {
      nh_private.param<float>("square/length", square_length, 5.0);
      pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
          "pose_topic", 100, &MotionPrimitive::setPose, motion_primitive_.get());
    } else {
      nh_private.param<float>("square/duration", duration, 10.0);
    }
    motion_primitive_ = std::make_unique<Square>(square_length, duration, speed, feedback_method);
  } else if (primitive_type == PrimitiveType::BOUSTROPHEDON) {
    float lawnmower_length, lawnmower_width, lawnmower_long_strip_duration,
        lawnmower_short_strip_duration;
    if (feedback_method == FeedbackMethod::POSE) {
      nh_private.param<float>("lawnmower/strip_length", lawnmower_length, 10.0);
      nh_private.param<float>("lawnmower/strip_width", lawnmower_width, 3.0);
      pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
          "pose_topic", 100, &MotionPrimitive::setPose, motion_primitive_.get());
    } else {
      nh_private.param<float>("lawnmower/long_strip_duration", lawnmower_long_strip_duration, 10.0);
      nh_private.param<float>(
          "lawnmower/short_strip_duration", lawnmower_short_strip_duration, 3.0);
    }
    motion_primitive_ = std::make_unique<LawnMower>(lawnmower_long_strip_duration,
                                                    lawnmower_short_strip_duration,
                                                    lawnmower_length,
                                                    lawnmower_width,
                                                    speed,
                                                    feedback_method);
  } else {
    ROS_FATAL("Invalid primitive type: %s", pattern.c_str());
    return 1;
  }

  std::function<void(const float)> depth_callback =
      std::bind(&MotionPrimitive::setDepth, motion_primitive_.get(), std::placeholders::_1);

  motion_primitive_->execute(num_of_runs);
  return EXIT_SUCCESS;
}