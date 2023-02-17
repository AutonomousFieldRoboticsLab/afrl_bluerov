#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>

#include "Primitives.h"
#include "Publisher.h"
#include "Utils.h"

ros::Publisher pub_rc_command;

void pressureCallback(const sensor_msgs::FluidPressure::ConstPtr& msg,
                      std::function<void(const double)> depth_callback,
                      double fluid_density,
                      std::string environment) {
  double depth = (msg->fluid_pressure - 101300.0f) / (fluid_density * 9.80665f);
  ROS_INFO_STREAM_DELAYED_THROTTLE(10, "Pressure: " << msg->fluid_pressure << "\tDepth: " << depth);

  depth_callback(depth);
}

void publishRCOverrideCommand(std::vector<int>& pwms) {
  mavros_msgs::OverrideRCIn rc_override;
  for (int i = 0; i < rc_override.channels.size(); ++i) {
    rc_override.channels[i] = 1500;
  }

  // TODO(bjoshi:) Looks like I have indexed the channels correctly
  // Use  as parameter later

  rc_override.channels[0] = pwms[0];
  rc_override.channels[1] = pwms[1];
  rc_override.channels[2] = pwms[2];
  rc_override.channels[3] = pwms[3];
  rc_override.channels[4] = pwms[4];
  rc_override.channels[5] = pwms[5];

  // For the lights
  rc_override.channels[6] = 1800;
  pub_rc_command.publish(rc_override);
}

void arm(ros::NodeHandle& nh, bool state) {
  ros::ServiceClient client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  client.waitForExistence();

  mavros_msgs::CommandBool mavros_cmd;
  mavros_cmd.request.value = state;
  client.call(mavros_cmd);
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
      std::bind(
          pressureCallback, std::placeholders::_1, depth_callback, fluid_density, environment));

  pub_rc_command = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);

  arm(nh, true);
  transect->setMotorCommandCallback(&publishRCOverrideCommand);

  // Wait for attitude and depth to be initialized for 10 secs
  while (ros::ok() && (!transect->is_attitude_initialized_ || !transect->is_depth_initialized_)) {
    ros::spinOnce();
    ros::Time start_time = ros::Time::now();
    ros::Rate rate(10);
    if ((ros::Time::now() - start_time).toSec() > 10.0) {
      ROS_ERROR("Attitude or depth not initialized");
      return false;
    }
  }

  // Execute transect
  Eigen::Vector3d initial_attitude = transect->current_attitude_;

  double initial_depth = transect->current_depth_;
  Eigen::Vector3d target_attitude =
      transect->current_attitude_ + Eigen::Vector3d(0.0, 0.0, M_PI_2 / 2.0);

  std::cout << target_attitude << std::endl;
  while (ros::ok() &&
         (transect->current_attitude_ - target_attitude).norm() > 10.0 / 180.0 * M_PI) {
    transect->executeGlobalAttitude(target_attitude);
    ros::spinOnce();
  }

  arm(nh, false);

  ROS_INFO_STREAM("Done executing transect!!. Enjoy ");
  return EXIT_SUCCESS;
}