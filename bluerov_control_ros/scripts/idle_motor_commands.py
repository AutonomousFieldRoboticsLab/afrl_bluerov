#!/usr/bin/env python

from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool
import rospy


def get_defualt_rc():
    rc = OverrideRCIn()
    for i in range(len(rc.channels)):
        rc.channels[i] = 1500
    return rc


def arm(state):
    rospy.wait_for_service("/mavros/cmd/arming")
    try:
        armService = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        armService(state)
    except rospy.ServiceException as e:
        rospy.loginfo("Service arm call failed: %s" % e)


if __name__ == "__main__":
    rospy.init_node("idle_motor_commands")
    pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)
    rate = rospy.Rate(10)

    arm(True)
    while not rospy.is_shutdown():
        pub.publish(get_defualt_rc())
        rate.sleep()
