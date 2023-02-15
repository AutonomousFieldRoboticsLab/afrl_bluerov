#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>

#include "Primitives.h"

// T0D0(bjoshi): Fix pressure to use callback directly to primitives

bool got_depth = false;
float depth = 0.0;

void pressureCallback(const sensor_msgs::FluidPressure::ConstPtr& msg,
                      std::function<void(const float)> depth_callback) {
  ROS_INFO_STREAM_THROTTLE(10, "Pressure: " << msg->fluid_pressure);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "bluerov_primitives");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::unique_ptr<MotionPrimitive> motion_primitive_ = nullptr;

  std::string pattern;
  std::string feedback_method_str;

  float speed;
  int num_of_runs;

  nh_private.param<std::string>("pattern", pattern, "square");
  nh_private.param<std::string>("feedback_type", feedback_method_str, "attitude_with_depth");
  nh_private.param<float>("speed", speed, 1.0);
  nh_private.param<int>("num_of_runs", num_of_runs, 1);

  PrimitiveType primitive_type = primitiveTypeFromString(pattern);
  FeedbackMethod feedback_method = feedbackMethodFromString(feedback_method_str);

  if (primitive_type == PrimitiveType::TRASECT) {
    float trasect_length, duration;
    // if we have acess to VIO Pose, we can execute the exact trasect length
    if (feedback_method == FeedbackMethod::POSE) {
      nh_private.param<float>("trasect/trasect_length", trasect_length, 5.0);
    } else {
      nh_private.param<float>("trasect/duration", duration, 5.0);
    }
    motion_primitive_ = std::make_unique<Trasect>(trasect_length, duration, speed, feedback_method);

  } else if (primitive_type == PrimitiveType::SQUARE) {
    // if we have acess to VIO Pose, we can execute the exact trasect length
    float square_length, duration;
    if (feedback_method == FeedbackMethod::POSE) {
      nh_private.param<float>("square/length", square_length, 5.0);
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

  ros::Subscriber attitude_sub = nh.subscribe<sensor_msgs::Imu>(
      "attitude_topic", 100, &MotionPrimitive::setAttitude, motion_primitive_.get());
  ros::Subscriber pressure_sub = nh.subscribe<sensor_msgs::FluidPressure>(
      "pressure_topic", 10, std::bind(pressureCallback, std::placeholders::_1, depth_callback));
  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "pose_topic", 100, &MotionPrimitive::setPose, motion_primitive_.get());

  while (ros::ok()) {
    ros::spinOnce();
  }
}