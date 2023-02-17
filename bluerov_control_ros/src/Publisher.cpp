#include "Publisher.h"

#include <mavros_msgs/OverrideRCIn.h>

#include "Utils.h"

Publisher::Publisher(ros::NodeHandle& nh) : nh_(nh) {
  nh_.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 100);
}

void Publisher::publishRCOverrideCommand(std::vector<int>& pwms) {
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

  pub_rc_override_.publish(rc_override);
}

mavros_msgs::OverrideRCIn Publisher::constructDefaultRCMessage() {
  mavros_msgs::OverrideRCIn rc_override;
  for (int i = 0; i < rc_override.channels.size(); ++i) {
    rc_override.channels[i] = 1500;
  }
}
