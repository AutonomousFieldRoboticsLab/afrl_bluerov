#!/usr/bin/env python
# vim:set ts=4 sw=4 et:
#
# Copyright 2014 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

from __future__ import print_function
from os import stat


import rospy
from sensor_msgs.msg import Joy

from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool

import numpy as np


def arduino_map(x, inmin, inmax, outmin, outmax):
    return (x - inmin) * (outmax - outmin) / (inmax - inmin) + outmin


def load_map(m, n):
    for k, v in m.items():
        m[k] = rospy.get_param(n + k, v)


def get_defualt_rc():
    rc = OverrideRCIn()
    for i in range(len(rc.channels)):
        rc.channels[i] = 1500
    return rc


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
        return "Channel: {}, Min: {}, Max: {}".format(self.chan, self.min, self.max)


def arm(state):
    rospy.wait_for_service("/mavros/cmd/arming")
    try:
        armService = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        armService(state)
    except rospy.ServiceException as e:
        rospy.loginfo("Service arm call failed: %s" % e)


def construct_rc_message(channel_values):
    rc_message = get_defualt_rc()

    for (index, value) in enumerate(channel_values):
        rc_message.channels[index] = value

    return rc_message


class JoyStick(object):
    def __init__(self):

        # Joystick button map
        self.axes_map = {
            "forward": 3,
            "lateral": 4,
            "yaw": 0,
            "throttle": 1,
            "lights": 2,
        }
        self.axes_scale = {
            "forward": 1.0,
            "lateral": 1.0,
            "yaw": 1.0,
            "throttle": 1.0,
            "lights": 1.0,
        }
        self.rc_channels = {
            "forward": RCChan("forward", 0),
            "lateral": RCChan("lateral", 1),
            "yaw": RCChan("yaw", 3),
            "throttle": RCChan("throttle", 2),
        }

        self.x_factor = [-1.0, -1.0, 1.0, 1.0, 0.0, 0.0]
        self.y_factor = [-1.0, 1.0, -1.0, 1.0, 0.0, 0.0]
        self.z_factor = [0.0, 0.0, 0.0, 0.0, -1.0, -1.0]

        self.roll_factor = [0.0, 0.0, 0.0, 0.0, 1.0, -1.0]
        self.pitch_factor = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.yaw_factor = [-1.0, 1.0, 1.0, -1.0, 0.0, 0.0]

        self.directions = [-1.0, -1.0, -1.0, 1.0, 1.0, 1.0]

        self.button_map = {"arm": 0, "disarm": 1}

        self.forward_minimum_pwms = [20, 20, 20, 20, 20, 20, 20]
        self.backward_minimum_pwms = [-20, -20, -20, -20, -20, -20]

        self.max_motor_speed = 150

        rospy.loginfo("RC Override Control")

        load_map(self.axes_map, "~axes_map/")
        print(self.axes_map)
        load_map(self.axes_scale, "~axes_scale/")
        print(self.axes_scale)
        load_map(self.button_map, "~button_map/")
        print(self.button_map)

        for _, rc_chan in self.rc_channels.items():
            rc_chan.load_param()

        self.defualt_rc_value = 1500.0
        self.max_pwm = 2000.0
        self.min_pwm = 1000.0

        self.light_value = self.min_pwm
        self.light_steps = 4
        self.light_button = 0.0

        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_cb)
        self.override_pub = rospy.Publisher(
            "/mavros/rc/override", OverrideRCIn, queue_size=10
        )
        # for name, rc_chan in self.rc_channels.items():
        #     print('Name: {} '.format(name), rc_chan)

    def convert_motor_intensities_to_pwms(self, intensities):
        offset_motor_speeds = [1500, 1500, 1500, 1500, 1500, 1500]

        for i in range(len(offset_motor_speeds)):
            offset = 0

            if intensities[i] < 0:
                offset = self.backward_minimum_pwms[i]
            elif intensities[i] > 0:
                offset = self.forward_minimum_pwms[i]

            offset_motor_speeds[i] = (
                offset_motor_speeds[i]
                + (intensities[i] * self.max_motor_speed)
                + offset
            )

        return offset_motor_speeds

    def get_axis(self, joy_msg, n):
        return joy_msg.axes[self.axes_map[n]] * self.axes_scale[n]

    def get_buttons(self, joy_msg, n):
        return joy_msg.buttons[self.button_map[n]]

    def set_chan(self, name, value):
        ch = self.rc_channels[name]
        self.rc_command.channels[ch.chan] = ch.calc_us(value)
        rospy.logdebug("RC%d (%s): %d us", ch.chan, ch.name, ch.calc_us(value))

    def joy_cb(self, joy_msg):
        # get axes normalized to -1.0..+1.0 RPY, 0.0..1.0 T
        forward = self.get_axis(joy_msg, "forward")
        lateral = self.get_axis(joy_msg, "lateral")
        yaw = self.get_axis(joy_msg, "yaw")
        throttle = self.get_axis(joy_msg, "throttle")

        thurst = [forward, lateral, throttle, 0.0, 0.0, yaw]
        motor_intensities = self.convert_thrust_vector_to_motor_intensities(thurst)

        motor_intensities = [
            intensity * direction
            for intensity, direction in zip(motor_intensities, self.directions)
        ]

        motor_pwms = self.convert_motor_intensities_to_pwms(motor_intensities)

        for speed in motor_pwms:
            assert (abs(speed) - 1500) <= 400

        rc_command = construct_rc_message(motor_pwms)

        rc_command.channels = [int(x) for x in rc_command.channels]

        # Handle lights

        light = self.get_axis(joy_msg, "lights")
        if light != self.light_button:
            self.light_button = light
            if light != 0.0:
                self.get_light_intensity(light)

        self.light_value = np.clip(self.light_value, self.min_pwm, self.max_pwm)

        # print('Light: {}'.format(self.light_value))

        rc_command.channels[6] = int(self.light_value)
        self.override_pub.publish(rc_command)

        if self.get_buttons(joy_msg, "arm"):
            arm(True)
        elif self.get_buttons(joy_msg, "disarm"):
            arm(False)

    def convert_thrust_vector_to_motor_intensities(self, thrust_vector):

        # thrust vector is: x,y,z,roll,pitch,yaw

        motor_intensities = [0.0] * 6

        max_intensity = 1.0

        # x,y,yaw intensities
        for i in range(len(motor_intensities)):
            motor_intensities[i] = (
                thrust_vector[0] * self.x_factor[i]
                + thrust_vector[1] * self.y_factor[i]
                + thrust_vector[2] * self.z_factor[i]
                + thrust_vector[3] * self.roll_factor[i]
                + thrust_vector[4] * self.pitch_factor[i]
                + thrust_vector[5] * self.yaw_factor[i]
            )

        if abs(motor_intensities[i]) > max_intensity:
            max_intensity = abs(motor_intensities[i])

        normalized_intensities = [val / max_intensity for val in motor_intensities]

        return normalized_intensities

    def get_light_intensity(self, joy_input):
        step = (self.max_pwm - self.min_pwm) / self.light_steps
        self.light_value = self.light_value + joy_input * step


if __name__ == "__main__":
    rospy.init_node("bluerov_teleop")
    # mavros.set_namespace(args.mavros_ns)

    jostick = JoyStick()
    while not rospy.is_shutdown():
        rospy.spin()
