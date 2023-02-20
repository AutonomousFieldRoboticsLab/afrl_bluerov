#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

#include "Primitives.h"

geometry_msgs::Quaternion attitude;
geometry_msgs::Point position;

bool attitude_initialized = false;
bool position_initialized = false;

void pressureCallback(const sensor_msgs::FluidPressure::ConstPtr& pressure_msg,
                      double fluid_density) {
  double depth = (pressure_msg->fluid_pressure - 101300.0f) / (fluid_density * 9.80665f);
  ROS_INFO_STREAM_DELAYED_THROTTLE(
      1, "Pressure: " << pressure_msg->fluid_pressure << "\tDepth: " << depth);
  position = geometry_msgs::Point();
  position.z = depth;
  position_initialized = true;
}

void attitudeCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  attitude = imu_msg->orientation;
  attitude_initialized = true;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  position = msg->pose.position;
  attitude = msg->pose.orientation;
  attitude_initialized = true;
  position_initialized = true;
}

void publishTf(const ros::TimerEvent& event) {
  if (!attitude_initialized) {
    ROS_WARN_STREAM("Attitude or position not initialized, not publishing tf");
    return;
  }

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(position.x, position.y, position.z));
  tf::Quaternion q(attitude.x, attitude.y, attitude.z, attitude.w);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "bluerov"));
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string environment;
  std::string feedback_method_str;
  nh_private.param<std::string>("feedback_method", feedback_method_str, "attitude_with_depth");

  double fluid_density;
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

  ros::Subscriber attitude_sub =
      nh.subscribe<sensor_msgs::Imu>("attitude_topic", 100, &attitudeCallback);
  ros::Subscriber pressure_sub = nh.subscribe<sensor_msgs::FluidPressure>(
      "pressure_topic", 10, std::bind(&pressureCallback, std::placeholders::_1, fluid_density));

  ros::Subscriber pose_sub;
  if (feedback_method == FeedbackMethod::POSE) {
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("pose_topic", 10, &poseCallback);
  }

  ros::Timer timer = nh.createTimer(ros::Duration(0.05), &publishTf);

  while (ros::ok()) {
    ros::spinOnce();
  }
}