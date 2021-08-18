#!/usr/bin/env python

from mavros_msgs.msg import OverrideRCIn
import rospy

default_pwm = 1500


def get_defualt_RC():
    rc = OverrideRCIn()
    for i in range(len(rc.channels)):
        rc.channels[i] = default_pwm
    return rc


if __name__ == '__main__':

    rospy.init_node('test_motors')
    pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

    rc = get_defualt_RC()

    rospy.loginfo("Publishing 1200 to Channel 0")
    rc.channels[0] = 1200
    pub.publish(rc)

    rospy.sleep(30)
    rc.channels[0] = default_pwm
    pub.publish(rc)
