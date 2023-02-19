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
  ros::init(argc, argv, "bluerov_lawnmower");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string feedback_method_str;
  float speed;

  nh_private.param<std::string>("feedback_method", feedback_method_str, "attitude_with_depth");
  nh_private.param<float>("speed", speed, 1.0);

  FeedbackMethod feedback_method = feedbackMethodFromString(feedback_method_str);

  float strip_length, strip_width, short_strip_duration, long_strip_duration;
  // if we have acess to VIO Pose, we can execute the exact transect length
  if (feedback_method == FeedbackMethod::POSE) {
    nh_private.param<float>("strip_length", strip_length, 10.0);
    nh_private.param<float>("strip_width", strip_width, 3.0);
  } else {
    nh_private.param<float>("short_strip_duration", short_strip_duration, 5.0);
    nh_private.param<float>("long_strip_duration", long_strip_duration, 20.0);
  }
  std::unique_ptr<MotionPrimitive> lawnmower = std::make_unique<LawnMower>(
      long_strip_duration, short_strip_duration, strip_length, strip_width, speed, feedback_method);
  pub_rc_command = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);

  arm(nh, true);
  lawnmower->setMotorCommandCallback(&publishRCOverrideCommand);

  lawnmower->execute();
  return EXIT_SUCCESS;
}