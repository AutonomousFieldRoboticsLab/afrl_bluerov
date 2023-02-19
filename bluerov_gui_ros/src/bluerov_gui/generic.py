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

import roslib  # roslib.load_manifest('aquamenu')
import rospy
import sys
import os
import time
#from aquacore.srv import *
#from aquacore.msg import *
from bluerov_gui_ros.srv import Menu, ChangeParameter
from std_srvs.srv import *
from std_msgs.msg import String
import subprocess
import traceback


class GenericMenu:
    '''
    The Super Awesome StereoRigMenu (from AquaMenu) TODO generalize it.

    Author: Sharmin Rahman

    This is the menu super class. You should subclass StereoRigMenu in another file and override
    getMenu method to return your own menu.

    '''

    def __init__(self):

        print ("Default menu starting.")
        rospy.init_node('expmanager')

        rospy.wait_for_service('/stereo_rig/ui/menu')
        self.updateMenu = rospy.ServiceProxy('/stereo_rig/ui/menu', Menu)

        self.record = rospy.ServiceProxy('/stereo_rig/record', Empty)
        # TODO more generic.
        self.slave1_exposure = rospy.ServiceProxy(
            '/stereo_rig/slave1/parameters', ChangeParameter)
        # TODO more generic.
        self.slave2_exposure = rospy.ServiceProxy(
            '/stereo_rig/slave2/parameters', ChangeParameter)

        print "Finished with service proxies."

        #self.command_pub = rospy.Publisher('/aqua/command', Command)
        self.msg_pub = rospy.Publisher(
            '/stereo_rig/ui/msg', String, queue_size=10)

        self.menu = self.getMenu()
        self.showMenu(self.menu)

    def getMenu(self):
        '''
        Keys of the dictionary are menu items. Each menu item is a tuple (i,j), where integer
        i specifies that the menu item should be listed as the i-th menu item, and string j
        indicates the text for the menu item.

        Values of the dictionary can be:
          - A dictionary denoting a submenu. Nest as many submenus as you want.
          - Special string '__back__' tells the menu to go back
          - A callable object (i.e. function name, lambda etc.), which will be executed when
            the menu item is selected

        Example:

        return {(0,'Aqua'): {
                  (0, '<<'    ) : '__back__',
                  (1, 'Calib' ) : self.calibrate_robot,
                  (2, 'Record'):{
                    (0, '<<'  )  : '__back__',
                    (1, '1min')  : lambda: self.runexp(["barbados2015_mosaic_collection", "record_bags.launch"], e_time=60,exec_once=True,show_timer=True),
                  }
                  (3, 'Gains') : {
                    (1, '1floor' : lambda: setattr(self, 'ship_sea_floor_depth', self.setvalue("floor", self.ship_sea_floor_depth, 0.5, 0 )),
                  }
                }
                (0, 'Another Menu...'): {
                  ...
                }
               }

        '''
        raise NotImplementedError

    # ensure the __back__ option is always the first option
    def menuOrderComparator(self, x, y, menu):
        if menu[x] == '__back__':
            return -1
        if menu[y] == '__back__':
            return 1
        return cmp(x, y)

    # recursive menu handling
    def showMenu(self, currMenu):
        while not rospy.is_shutdown():
            # self.show_depth(False)
            menuTuples = sorted(currMenu.keys(), key=lambda x: x[0])
            menuItems = [i[1] for i in menuTuples]
            print "Menu items: ", menuItems
            print "waiting for response:"
            choice = self.updateMenu("", menuItems, 0, 0).selection
            print "Choice: ", choice
            selection = currMenu[menuTuples[choice]]
            if hasattr(selection, '__call__'):
                try:
                    selection()
                except:
                    print "Exception in user code:"
                    print '-'*60
                    traceback.print_exc(file=sys.stdout)
                    print '-'*60
            elif (selection == '__back__'):
                return
            else:
                self.showMenu(selection)

    # waits for t seconds or until the process subproc terminates
    def clock(self, t, subproc=None, show_timer=True):
        while t > 0 and not rospy.is_shutdown():
            if show_timer:
                self.msg_pub.publish(String(str(t)))
            rospy.sleep(1.0)
            t = t-1
            if subproc is not None:
                if subproc.poll() is not None:
                    rospy.loginfo("clock: Subprocess ended")
                    t = 0
                    return False
        return True

    # Launches the roslaunch file specified by launchstr ( "package_name launc_file_name").
    # Waits for e_time or until the launch process terminates. If exec_once is False, it will
    # display a menu for repeating the experiment
    def runexp(self, launchstr, e_time=180, exec_once=False, show_timer=True):
        #rospy.lxsoginfo("runexp: launchstr %s, e_time %d, exec_once %d"%(launchstr,e_time,exec_once))
        # self.aqua_pause(False)
        # self.show_depth(False)
        self.show_time(False)
        while not rospy.is_shutdown():
            launcher = subprocess.Popen(["roslaunch"] + launchstr)
            self.clock(e_time, launcher, show_timer)
            rospy.loginfo("runexp: Subprocess ended")
            if launcher.poll() is None:
                launcher.terminate()
                launcher.wait()
            if not exec_once:
                choice = self.updateMenu(
                    "Rept?", ["Yes", "No"], 0, 0).selection
                if choice == 1:
                    break
            else:
                break
        rospy.loginfo("runexp: done")
        self.show_time(False)

    def setvalue(self, valname, initval, increment, minval=0):
        print 'initval: ', initval
        print 'increment: ', increment

        v = initval
        while not rospy.is_shutdown():
            subchoice = self.updateMenu(
                valname+"="+str(v), ["--", "++", "OK"], 0, 0).selection
            if subchoice == 0:
                v = max(minval, v-increment)
            elif subchoice == 1:
                v += increment
            elif subchoice == 2:
                break
            elif subchoice == 3:
                v = initval
                break
        return v


if __name__ == '__main__':
    print 'default_menu main'
    stereoRigMenu = StereoRigMenu()
