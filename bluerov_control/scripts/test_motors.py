#!/usr/bin/env python

from mavros_msgs.msg import OverrideRCIn
import rospy

default_pwm = 1500

yaw_factor = [0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0]
x_factor = [0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
y_factor = [0.0,  0.0, 0.0, 0.0, -1.0, 0.0, -1.0, 0.0]

roll_factor = [1.0, 0.0, -1.0, -1.0, 0.0, -1.0, 0.0, 0.0]
pitch_factor = [-1.0, 0.0, -1.0, 1.0, 0.0, -1.0, 0.0, 0.0]
z_factor = [1.0, 0.0, -1.0, 1.0, 0.0, 1.0, 0.0, 0.0]


def get_defualt_rc():
    rc = OverrideRCIn()
    for i in range(len(rc.channels)):
        rc.channels[i] = default_pwm
    return rc


if __name__ == '__main__':

    rospy.init_node('test_motors')
    pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

    rc = get_defualt_rc()

    rospy.loginfo("Publishing 1350 to Channel 0")
    rc.channels[0] = 1450

    print(rc)

    # rospy.sleep(30)
    # rc.channels[0] = default_pwm
    # pub.publish(rc)

    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        print('Publishing RC: {}'.format(rc))
        pub.publish(rc)
        rate.sleep()
