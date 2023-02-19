#!/usr/bin/env python

from mavros_msgs.msg import OverrideRCIn
import rospy

def get_defualt_rc():
    rc = OverrideRCIn()
    for i in range(len(rc.channels)):
        rc.channels[i] = 1500
    return rc


if __name__=="__main__":
    rospy.init_node("idle_motor_commands")
    pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(get_defualt_rc())
        rate.sleep()
    
