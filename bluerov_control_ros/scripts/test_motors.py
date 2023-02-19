#!/usr/bin/env python

from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool

import rospy
import argparse

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

def arm(state):
    rospy.wait_for_service("/mavros/cmd/arming")
    try:
        armService = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        armService(state)
    except rospy.ServiceException as e:
        rospy.loginfo("Service arm call failed: %s" % e)

if __name__ == '__main__':

    parser = argparse.ArgumentParser(
                    prog = 'TestMotors',
                    description = 'Test each motor direction')
    
    parser.add_argument('-m', '--motors')      # option that takes a value
    args = parser.parse_args()

    arm(True)
    rospy.init_node('test_motors')
    pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

    rc = get_defualt_rc()
    
    print("Testing motor direction for motor" , args.motors)      
    rospy.loginfo("Publishing 1600 to Channel 0")
    motor_num = int(args.motors)
    rc.channels[motor_num] = 1600

    print(rc)

    # rospy.sleep(30)
    # rc.channels[0] = default_pwm
    # pub.publish(rc)

    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        print('Publishing RC: {}'.format(rc))
        pub.publish(rc)
        rate.sleep()
