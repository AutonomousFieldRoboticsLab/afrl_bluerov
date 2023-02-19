#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>

#include "Primitives.h"
#include "Publisher.h"
#include "Utils.h"

ros::Publisher pub_rc_command;

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
  rc_override.channels[6] = 2000;
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
  float speed;

  nh_private.param<std::string>("feedback_method", feedback_method_str, "attitude_with_depth");
  nh_private.param<float>("speed", speed, 1.0);

  FeedbackMethod feedback_method = feedbackMethodFromString(feedback_method_str);

  float transect_length, duration;
  // if we have acess to VIO Pose, we can execute the exact transect length
  if (feedback_method == FeedbackMethod::POSE) {
    nh_private.param<float>("length", transect_length, 5.0);
  } else {
    nh_private.param<float>("duration", duration, 5.0);
  }
  transect = std::make_unique<Transect>(transect_length, duration, speed, feedback_method);
  pub_rc_command = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);

  arm(nh, true);
  transect->setMotorCommandCallback(&publishRCOverrideCommand);

  transect->execute();
  return EXIT_SUCCESS;
}