#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>

#include "Primitives.h"

void pressureCallback(const sensor_msgs::FluidPressure::ConstPtr& msg,
                      std::function<void(const double)> depth_callback,
                      double fluid_density) {
  ROS_INFO_STREAM_THROTTLE(10, "Pressure: " << msg->fluid_pressure);
  double depth = (msg->fluid_pressure - 101300.0f) / (fluid_density * 9.80665f);
  depth_callback(depth);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "bluerov_transect");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::unique_ptr<MotionPrimitive> transect = nullptr;

  std::string feedback_method_str;

  std::string environment;

  float fluid_density;
  float speed;

  nh_private.param<std::string>("feedback_method", feedback_method_str, "attitude_with_depth");
  nh_private.param<float>("speed", speed, 1.0);
  nh_private.param<std::string>("environment", environment, "sea");

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

  float transect_length, duration;
  // if we have acess to VIO Pose, we can execute the exact transect length
  if (feedback_method == FeedbackMethod::POSE) {
    nh_private.param<float>("length", transect_length, 5.0);
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "pose_topic", 100, &Transect::setPose, transect.get());
  } else {
    nh_private.param<float>("duration", duration, 5.0);
  }
  transect = std::make_unique<Transect>(transect_length, duration, speed, feedback_method);

  std::function<void(const float)> depth_callback =
      std::bind(&Transect::setDepth, transect.get(), std::placeholders::_1);

  ros::Subscriber attitude_sub =
      nh.subscribe<sensor_msgs::Imu>("attitude_topic", 100, &Transect::setAttitude, transect.get());
  ros::Subscriber pressure_sub = nh.subscribe<sensor_msgs::FluidPressure>(
      "pressure_topic",
      10,
      std::bind(pressureCallback, std::placeholders::_1, depth_callback, fluid_density));

  transect->execute(1);
  while (ros::ok()) {
    ros::spinOnce();
  }
}