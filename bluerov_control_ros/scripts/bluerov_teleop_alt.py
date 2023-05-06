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


def construct_rc_message(channel_values):
    rc_message = get_defualt_rc()

    for (index, value) in enumerate(channel_values):
        rc_message.channels[index] = value

    return rc_message


class JoyStick(object):
    def __init__(self):

        # Joystick button map (X-mode for Logitech F710 X-mode)
        # -----------------------------------------------------,
        self.axes_map = {
            "move_forward": 5, # Left Trigger
            "move_backward": 2, # Right Trigger
            "move_left_right": 0, # Left thumbstick X-axis
            "ascend_descend": 1, # Left thumbstick Y-axis
            "yaw": 3, # Right thumbstick X-axis
            "pitch": 4, # Right thumbstick Y-axis
            "lights": 7, # D-pad up-down
        }

        self.button_map = {
            "disarm": 6, # Back
            "arm": 7, # Start
            "roll_ccw": 4, # Left Index button.
            "roll_cw": 5 # Right Index button.
        }

        self.axes_scale = {
            "move_forward": 1.0,
            "move_backward": 1.0,
            "move_left_right": 1.0,
            "ascend_descend": 1.0,

            "yaw": 1.0,
            "pitch": 1.0,
            "lights": 1.0,
        }

        self.roll_thrust_factor = 0.3
        # -----------------------------------------------------'

        rospy.loginfo("Axes map: %s", self.axes_map)
        rospy.loginfo("Button map: %s", self.button_map)



        # MOTOR INDICES: [FRONT_RIGHT, FRONT_LEFT, BACK_RIGHT, BACK_LEFT, MIDDLE_RIGHT, MIDDLE_LEFT]
        self.x_factor = [-1.0, -1.0, 1.0, 1.0, 0.0, 0.0]
        self.y_factor = [-1.0, 1.0, -1.0, 1.0, 0.0, 0.0]
        self.z_factor = [0.0, 0.0, 0.0, 0.0, -1.0, -1.0]

        self.roll_factor = [0.0, 0.0, 0.0, 0.0, 1.0, -1.0]
        self.pitch_factor = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.yaw_factor = [-1.0, 1.0, 1.0, -1.0, 0.0, 0.0]

        self.directions = [-1.0, -1.0, -1.0, 1.0, 1.0, 1.0]

        self.forward_minimum_pwms = [20, 20, 20, 20, 20, 20, 20]
        self.backward_minimum_pwms = [-20, -20, -20, -20, -20, -20]

        self.max_motor_speed = 150

        self.defualt_rc_value = 1500.0
        self.max_pwm = 2000.0
        self.min_pwm = 1000.0

        self.light_pwm_min = 1000
        self.light_pwm_max = 2000

        # PWM control does not work for now.
        # self.light_max_step = 4
        self.light_max_step = 1

        self.light_curr_step = 0
        self.light_axis_last_value = None

        self.move_forward_initialized = False
        self.move_backward_initialized = False

        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_cb)
        self.override_pub = rospy.Publisher(
            "/mavros/rc/override", OverrideRCIn, queue_size=10
        )

        self.last_thrust = None

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

    def joy_cb(self, joy_msg):
        # Trigger buttons give 1.0 on resting position. The range is from +1.0 to -1.0.
        move_forward = abs(self.get_axis(joy_msg, "move_forward") - 1.0) / 2.0
        if not self.move_forward_initialized:
            if move_forward == 0.5:
                move_forward = 0.0
            else:
                self.move_forward_initialized = True

        move_backward = abs(self.get_axis(joy_msg, "move_backward") - 1.0) / 2.0
        if not self.move_backward_initialized:
            if move_backward == 0.5:
                move_backward = 0.0
            else:
                self.move_backward_initialized = True

        x_thrust = (move_forward - move_backward)
        y_thrust = self.get_axis(joy_msg, "move_left_right")
        z_thrust = self.get_axis(joy_msg, "ascend_descend")


        roll = 0.0
        if self.get_buttons(joy_msg, "roll_ccw") > 0.0:
            roll = 1.0 * self.roll_thrust_factor
        elif self.get_buttons(joy_msg, "roll_cw") > 0.0:
            roll = -1.0 * self.roll_thrust_factor

        pitch = 0.0
        yaw = self.get_axis(joy_msg, "yaw")
        thrust = [x_thrust, y_thrust, z_thrust, roll, pitch, yaw]

        if (not self.last_thrust) or (self.last_thrust != thrust):
            rospy.loginfo("Thrust: %s", thrust)
            self.last_thrust = thrust


        # thrust = [forward, lateral, throttle, 0.0, 0.0, yaw]
        motor_intensities = self.convert_thrust_vector_to_motor_intensities(thrust)

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
        # --------------------------------------------------------------,
        light_axis_value = self.get_axis(joy_msg, "lights")
        if (self.light_axis_last_value is None) or (self.light_axis_last_value != light_axis_value):
            if light_axis_value > 0.0:
                self.light_curr_step += 1
            elif light_axis_value < 0.0:
                self.light_curr_step -= 1

            self.light_curr_step = np.clip(self.light_curr_step, 0, self.light_max_step)
            self.light_axis_last_value = light_axis_value

        light_value = self.get_light_intensity()
        # rospy.loginfo('Light: %s', light_value)
        rc_command.channels[6] = int(light_value)
        # --------------------------------------------------------------'

        self.override_pub.publish(rc_command)

        if self.get_buttons(joy_msg, "arm"):
            rospy.loginfo("Arming ... ")
            arm(True)
        elif self.get_buttons(joy_msg, "disarm"):
            rospy.loginfo("Disarming ... ")
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

    def get_light_intensity(self):
        intensity_range = (self.light_pwm_max - self.light_pwm_min)
        return self.light_pwm_min + (self.light_curr_step * intensity_range) / self.light_max_step


if __name__ == "__main__":
    rospy.init_node("bluerov_teleop")
    # mavros.set_namespace(args.mavros_ns)

    jostick = JoyStick()
    while not rospy.is_shutdown():
        rospy.spin()
