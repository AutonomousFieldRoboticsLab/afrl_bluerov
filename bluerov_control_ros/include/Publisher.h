#pragma once

#include <mavros_msgs/OverrideRCIn.h>
#include <ros/ros.h>

class Publisher {
 public:
  Publisher(ros::NodeHandle& nh_private);
  ~Publisher() = default;

  void publishRCOverrideCommand(std::vector<int>& pwms);

  ros::Publisher pub_rc_override_;
  ros::NodeHandle nh_;

 private:
  mavros_msgs::OverrideRCIn constructDefaultRCMessage();
};