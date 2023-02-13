#!/usr/bin/env python
################################################################################
# DO NOT MODIFY - AUTO-GENERATED
#
# Copyright (c) 2016, McGill University / Independent Robotics Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
################################################################################

import subprocess

from generic import GenericMenu
import os
import sensor_msgs.msg
import time
import rospy
from std_msgs.msg import String


class BlueROVMenu(GenericMenu):
    def __init__(self):
        # Initialize instance variables here...
        self.testValue = 20
        #self.camera_back_sub = rospy.Subscriber('/camera_back/image_raw', sensor_msgs.msg.Image, self.handle_cam_msg)
        GenericMenu.__init__(self)

    def handle_cam_msg(self, msg):
        pass

    def action_svin_start(self):
        print("SVIN-start action not implemented yet.")

    def action_svin_stop(self):
        print("SVIN-stop action not implemented yet.")

    def action_square_timed(self):
        print("Square-timed action not implmented yet.")

    def action_square_svin(self):
        print("Square-svin action not implemented yet.")

    def action_lawnmower_timed(self):
        print("Lawnmower-timed action not implmented yet.")

    def action_lawnmower_svin(self):
        print("Lawnmower-svin action not implemented yet.")

    def action_record_start(self):
        print("Record-start action not implemented yet.")

    def action_record_stop(self):
        print("Record-stop action not implemented yet.")


    def getMenu(self):
        small_trajec_time = 100
        large_trajec_time = 290
        medium_line_time = 70

        small_lawnmower_time = 135
        small_ret_lawnmower_time = small_lawnmower_time + 90
        medium_lawnmower_time = 285
        large_lawnmower_time = 440

        menu = {
            (0, 'SVIN'): {
                (0, '<<'): '__back__',
                (1, 'Start'): self.action_svin_start,
                (2, 'Stop'): self.action_svin_stop,
            },

            (1, 'Square'): {
                (0, '<<'): '__back__',
                (1, 'Timed'): self.action_square_timed,
                (2, 'SVIN'): self.action_square_svin,
            },

            (2, 'Lawnmower'): {
                (0, '<<'): '__back__',
                (1, 'Timed'): self.action_lawnmower_timed,
                (1, 'SVIN'): self.action_lawnmower_svin,
            },

            (3, 'Record'): {
                (0, '<<'): '__back__',
                (1, 'Start'): self.action_record_start,
                (2, 'Stop'): self.action_record_stop,
            },
        }


        """    
        menu = {
            (1, 'StereoRigTroubleShoot'): {
            (1, '<<'): '__back__',
            (2, 'Relaunch nodes'): lambda: subprocess.Popen(["rosrun", "stereo_rig_bringup", "startup_stereo.sh"]),
            (3, 'Shutdown'): lambda: os.fork(subprocess.Popen(["sudo", "shutdown", "-hP", "now"],
                                                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT))
        },  # Ends StereoRigTroubleShoot top-level menu

            (2, 'Record'): self.record,
            (3, 'slave1'): {
            (1, '<<'): '__back__',
            (2, "Exposure"): {
                (1, '<<'): '__back__',
                (2, "Increment"): lambda: self.slave1_exposure("/slave1", "exposure", True),
                (3, "Decrement"): lambda: self.slave1_exposure("/slave1", "exposure", False),
            }
        }

        }

        """
        return menu


if __name__ == '__main__':
    print 'default_menu main'
    BlueROVMenu = BlueROVMenu()
