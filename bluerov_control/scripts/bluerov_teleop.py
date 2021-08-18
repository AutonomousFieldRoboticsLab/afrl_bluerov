#!/usr/bin/env python
# vim:set ts=4 sw=4 et:
#
# Copyright 2014 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

from __future__ import print_function

import argparse
import sys

import rospy
from geometry_msgs.msg import (Point, PoseStamped, Quaternion, TwistStamped,
                               Vector3)
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Header
from tf.transformations import quaternion_from_euler

import mavros
from mavros import command
from mavros import setpoint as SP
from mavros_msgs.msg import OverrideRCIn


def arduino_map(x, inmin, inmax, outmin, outmax):
    return (x - inmin) * (outmax - outmin) / (inmax - inmin) + outmin


def load_map(m, n):
    for k, v in m.items():
        m[k] = rospy.get_param(n + k, v)


class RCChan(object):
    def __init__(self, name, chan, min_pos=-1.0):
        self.name = name
        self.chan = chan
        self.min = 1000
        self.max = 2000
        self.min_pos = min_pos

    def load_param(self):
        self.chan = rospy.get_param("~rc_map/" + self.name, self.chan)
        self.min = rospy.get_param("~rc_min/" + self.name, self.min)
        self.max = rospy.get_param("~rc_max/" + self.name, self.max)

    def calc_us(self, pos):
        # warn: limit check
        return arduino_map(pos, self.min_pos, 1.0, self.min, self.max)

    def __str__(self):
        return 'Channel: {}, Min: {}, Max: {}'.format(self.chan, self.min, self.max)


def arm(state):
    try:
        command.arming(value=state)
    except rospy.ServiceException as ex:
        fault(ex)

    if not ret.success:
        rospy.loginfo("Request failed.")
    else:
        rospy.loginfo("Request success.")


class JoyStick(object):
    def __init__(self, name):

        # Joystick button map
        self.axes_map = {'forward': 3,
                         'lateral': 4,
                         'yaw': 0,
                         'throttle': 1
                         }
        self.axes_scale = {'forward': 1.0,
                           'lateral': 1.0,
                           'yaw': 1.0,
                           'throttle': 1.0
                           }
        self.rc_channels = {
            'forward': RCChan('forward', 0),
            'lateral': RCChan('lateral', 1),
            'yaw': RCChan('yaw', 3),
            'throttle': RCChan('throttle', 2)
        }

        self.button_map = {'arm': 0, 'disarm': 1}

        rospy.loginfo('RC Override Control')

        load_map(self.axes_map, '~axes_map/')
        print(self.axes_map)
        load_map(self.axes_scale, '~axes_scale/')
        print(self.axes_scale)

        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_cb)
        self.override_pub = rospy.Publisher(
            '/mavros/rc/override', OverrideRCIn, queue_size=10)

        self.rc_command = OverrideRCIn()

        for name, rc_chan in self.rc_channels.items():
            rc_chan.load_param()

        self.defualt_rc_value = 1500

        # for name, rc_chan in self.rc_channels.items():
        #     print('Name: {} '.format(name), rc_chan)

    def get_axis(self, joy_msg, n):
        return joy_msg.axes[self.axes_map[n]] * self.axes_scale[n]

    def get_buttons(self, joy_msg, n):
        return j.buttons[self.button_map[n]]

    def set_chan(self, name, value):
        ch = self.rc_channels[name]
        self.rc_command.channels[ch.chan] = ch.calc_us(value)
        rospy.logdebug("RC%d (%s): %d us", ch.chan, ch.name, ch.calc_us(value))

    def joy_cb(self, joy_msg):
        # get axes normalized to -1.0..+1.0 RPY, 0.0..1.0 T
        forward = self.get_axis(joy_msg, 'forward')
        lateral = self.get_axis(joy_msg, 'lateral')
        yaw = self.get_axis(joy_msg, 'yaw')
        throttle = self.get_axis(joy_msg, 'throttle')

        # rospy.loginfo("RPYT: %f, %f, %f, %f", forward, lateral, yaw, throttle)

        self.rc_command = OverrideRCIn()

        for i in range(len(self.rc_command.channels)):
            self.rc_command.channels[i] = self.defualt_rc_value

        self.set_chan('forward', forward)
        self.set_chan('lateral', lateral)
        self.set_chan('yaw', yaw)
        self.set_chan('throttle', throttle)

        self.override_pub.publish(self.rc_command)


if __name__ == '__main__':
    rospy.init_node("bluerov_teleop")
    # mavros.set_namespace(args.mavros_ns)

    jostick = JoyStick('joystick')
    while not rospy.is_shutdown():
        rospy.spin()
