#!/usr/bin/python
"""GUI for Stereo rig.
TODO(aql) set some macros.
TODO(aql) make reading more general.
"""

# Standard libraries.
import functools  # To associate parameters with function.
import os  # For path handling and processes.
import signal  # To properly kill Qt and ROS.
import subprocess  # To launch processes.
import sys  # argv and exit.
import time  # Time conversion to string.

# External libraries.
import cv2  # For resize.
import numpy

# User Interface related libraries (ROS bindings).
from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal

# ROS related libraries.
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import dynamic_reconfigure.client
import dynamic_reconfigure.msg
from dynamic_reconfigure.srv import Reconfigure
import rosparam
import rospkg
import rospy
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temperature

# Libraries to import for Menu and Tags
import roslib
import rospy
import sys
import os
import time

from std_srvs.srv import *
from std_msgs.msg import String, UInt32MultiArray
import subprocess
import traceback
from bluerov_gui_ros.srv import Menu, ChangeParameter


# Default values.
CAMERA_NAME = "/bluerov"
IMAGE_RAW_STRING = "image_raw"
CAMERA_NODE = '/bluerov/spinnaker_camera_nodelet/'
TAG_TOPIC = '/bluerov/aruco_marker_publisher/markers_list'

LAUNCH_BAG_FILE = "barbados2017_logger.launch"
MESSAGE_TIMEOUT = 1.0
DECIMAL_PLACES = 1  # Decimal places after the dot in floats.
METER_TO_FOOT = 3.28084  # 1 m = 3.28084 ft.
DEPTH_IN_FEET = True  # Convert value from sensor in feet.

DATA_LABEL_FORMAT = '{} ({})'

DEPTH_STRING = "depth"
METER_STRING = "m"
FEET_STRING = "ft"

TEMPERATURE_STRING = "temp."
CELSIUS_STRING = "C"

PRESSURE_STRING = "press."
MBAR_STRING = "mbar"

EXPOSURE_STRING = "exp."
MILLISECOND_STRING = "ms"

EXPOSURE_FORMAT = "C1: {0:.1f}, C2: {1:.1f}"

IMAGE_UPDATE_INTERVAL = 0.1

# Menu and Tags Related #TODO not global variables!


class RosHandler(QtCore.QObject):
    '''Handling ROS callbacks to emit signal to update GUI from main thread.'''

    '''
    This is also the menu super class. You should subclass StereoRigMenu in another file
    and override getmenu to return your own menu.
    '''
    # Signals to emit to the GUI.
    # Necessary so that GUI does not freeze.
    image_updated = Signal(numpy.ndarray, str)
    diagnostic_updated = Signal(float, str)
    log_updated = Signal(str)
    cameras_updated = Signal(str)

    def __init__(self, launch_bag_file, camera_name, camera_node):
        # Initialize the RosHandler as a QObject
        QtCore.QObject.__init__(self)
        # Converter for images.
        self.bridge = CvBridge()

        # Launch file for start the bagging.
        self.launch_bag_file = launch_bag_file

        # Camera names (should match what is in header).
        self.camera_name = camera_name

        # Dynamic reconfiguration of nodes.
        # TODO(aql) more general for arbitrary nodes.
        self.camera_parameters_client = dynamic_reconfigure.client.Client(
            camera_node)

        # self.get_camera_parameters()
        self.start_recording_ar_first_time = None
        self.start_recording_ar_last_time = None
        self.started_recording = False

        # set initial exposure value
        self.get_camera_parameters()  # FIXME

        # Timers for messages.
        self.last_image_time = None
        self.last_imu_time = None

        # Menu variables as from the Aqua
        self.tag_id = -1
        self.question = ""
        self.message = ""
        self.message_time = None
        self.display_tags = True

        self.ros_initialization()

    def ros_initialization(self):
        # initialize node.
        rospy.init_node('bluerov_gui')

        # Subscribe to topics
        # Diagnostic topics.
        rospy.Subscriber("/bluerov/image_raw", Image,
                         self.image_callback, queue_size=1)

        # rospy.Subscriber("bar30/temperature", Temperature,
        #                  self.temperature_value_callback, queue_size=1)
        # rospy.Subscriber("bar30/depth", Depth,
        #                  self.depth_value_callback, queue_size=1)
        # rospy.Subscriber("bar30/pressure", FluidPressure,
        #                  self.pressure_value_callback, queue_size=1)
        # rospy.Subscriber("imu/imu", Imu, self.health_callback, queue_size=1)

        # Topic for AR tag menu.
        # rospy.Subscriber(TAG_TOPIC, UInt32MultiArray,
        #                  self.ar_callback, queue_size=5)

        # START Menu from Aqua
        # Initializing related with tags and menu
        self.tag_id = -1
        self.display_tags = False

        # Advertise Services
        rospy.Service("/stereo_rig/ui/menu", Menu, self.menu)
        rospy.Service("/stereo_rig/record", Empty, self.handle_record_bag)
        rospy.Timer(rospy.Duration(2), self.refresh_display)

        rospy.Subscriber("/stereo_rig/ui/msg", String,
                         self.handle_message, queue_size=1)
        rospy.Service("/stereo_rig/slave1/parameters",
                      ChangeParameter, self.change_parameter)
        # END Menu from Aqua

    def message_initialization(self):
        """Initializes GUI with messages.
        """
        # State of recording.
        self.diagnostic_updated.emit(self.started_recording, "recording")

        # Exposure parameters of camera.
        # exp1 = rosparam.get_param('ueye_cam_nodelet_slave1/exposure')
        # exp2 = rosparam.get_param('ueye_cam_nodelet_slave2/exposure')
        # self.cameras_updated.emit(EXPOSURE_FORMAT.format(exp1, exp2))

    def image_callback(self, image_msg):
        """Get image and display it. TODO(aql) more complete documentation.
        """
        try:
            # Update image on GUI only if a certain amount of time has passed.
            change_image = False

            # TODO(bjoshi): check camera frame id matches camera name

            if not self.last_image_time or image_msg.header.stamp - self.last_image_time > rospy.Duration.from_sec(IMAGE_UPDATE_INTERVAL):
                change_image = True
                self.last_image_time = image_msg.header.stamp
            if change_image:
                cv_image = None
                if image_msg.encoding == "mono8":
                    cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'mono8')
                else:
                    cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'rgb8')

                # Resize moved to GUI Dialog class as we can access the size of image label there
                # cv_image = cv2.resize(cv_image, IMAGE_SIZE)
                self.image_updated.emit(cv_image, image_msg.header.frame_id)
        except CvBridgeError as e:
            print(e)

    def depth_value_callback(self, value_msg):
        """Get value and display it. TODO(aql) more complete documentation.
        """
        depth = value_msg.depth
        self.diagnostic_updated.emit(depth, "depth")

    def temperature_value_callback(self, value_msg):
        """Get value and display it. TODO(aql) more complete documentation.
        """
        self.diagnostic_updated.emit(value_msg.temperature, "temperature")

    def pressure_value_callback(self, value_msg):
        """Get value and display it. TODO(aql) more complete documentation.
        """
        self.diagnostic_updated.emit(value_msg.fluid_pressure, "pressure")

    def health_callback(self, msg):
        """Get health information about topics. TODO(aql) more complete documentation.
        """
        t_sec = msg.header.stamp.to_sec()
        # + '.%03d' % (msg.header.stamp.nsecs / 1000000)
        msg_current_time = time.strftime('%H:%M:%S', time.localtime(t_sec))
        if "Imu" in type(msg).__name__:
            if not self.last_imu_time or \
                    msg.header.stamp - self.last_imu_time > rospy.Duration.from_sec(IMAGE_UPDATE_INTERVAL):
                self.diagnostic_updated.emit(t_sec, "imu")
                # self.change_diagnostic_value(t_sec, "imu")
                self.last_imu_time = msg.header.stamp
        elif "Range" in type(msg).__name__:
            if not self.last_sonar_time or \
                    msg.header.stamp - self.last_sonar_time > rospy.Duration.from_sec(IMAGE_UPDATE_INTERVAL):
                self.diagnostic_updated.emit(t_sec, "sonar")
                # self.change_diagnostic_value(t_sec, "sonar")
                self.last_sonar_time = msg.header.stamp

    def ar_callback(self, markers_msg):
        """Menu associated to AR tags. TODO(aql) more complete documentation.
        """
        for marker in markers_msg.data:
            rospy.loginfo(marker)

            # FIXME: show the corresponding operation  based on the value of a
            # tag
            # self.logger_label.setText("tag id: " + str(marker.id))
            if True:
                # self.perform_ar_process(markers_msg, log_msg) #FIXME: log_msg array of the messages to show in the logger
                if self.start_recording_ar_first_time is None:
                    rospy.loginfo("setting time")
                    self.start_recording_ar_first_time = markers_msg.header.stamp
                    self.start_recording_ar_last_time = markers_msg.header.stamp
                else:
                    if markers_msg.header.stamp - self.start_recording_ar_last_time > \
                            rospy.Duration.from_sec(1):  # TODO constant or param for duration.
                        # Reset timer as tag was not continuously visible.
                        self.start_recording_ar_first_time = None
                    else:
                        current_difference_time = markers_msg.header.stamp - \
                            self.start_recording_ar_first_time
                        self.log_msg = current_difference_time.to_sec()
                        rospy.loginfo(current_difference_time)
                        if current_difference_time > rospy.Duration.from_sec(2):
                            self.log_updated.emit(str(self.log_msg))
                        if current_difference_time > \
                                rospy.Duration.from_sec(5):
                            if marker.id == 0:
                                self.check_record_bag()
                                self.diagnostic_updated.emit(
                                    self.started_recording, "recording")
                                self.start_recording_ar_first_time = None
                            elif marker.id == 1:
                                self.start_recording_ar_first_time = None
                                self.log_msg = "The relaunch is started..."
                                self.log_updated.emit(str(self.log_msg))
                                self.relaunch_nodes()
                            elif marker.id == 3:
                                self.log_msg = "The computer is shutting down..."
                                self.log_updated.emit(str(self.log_msg))
                                self.shutdown_computer()
                                '''
                                self.log_msg = "The computer is rebooting..."
                                self.logger_label.setText(str(self.log_msg))
                                self.reboot_computer()
                                '''
                            else:
                                self.start_recording_ar_first_time = None
                        if marker.id == 2:
                            if current_difference_time > \
                                    rospy.Duration.from_sec(1.5):
                                val = self.reduce_exposure()
                                rospy.loginfo("--------------------EXPOSURE-----------------------\
                                " + EXPOSURE_FORMAT.format(val[0], val[1]))
                                self.start_recording_ar_first_time = None
                    self.start_recording_ar_last_time = markers_msg.header.stamp
            # TODO associate commands with tags.

    def get_camera_parameters(self):
        """Get current camera_parameters of the two camera nodes.
            TODO(aql) more complete documentation.
        """
        self.camera_parameters = self.camera_parameters_client.get_configuration(
            timeout=1.0)

    def set_camera_parameters(self, parameter):
        """Change current camera_parameters. TODO(aql) more complete documentation.
            parameter should be already in the format of a dictionary.
        """
        self.camera_left_parameters_client.update_configuration(parameter)
        self.camera_right_parameters_client.update_configuration(parameter)

    def relaunch_nodes(self):
        rospy.loginfo("relaunch the nodes")
        subprocess.Popen(["rosrun", "stereo_rig_bringup", "startup_stereo.sh"])

    def shutdown_computer(self):
        rospy.loginfo("shutdown computer")
        os.fork(subprocess.Popen(["sudo", "shutdown", "-hP", "now"],
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT))

    def reduce_exposure(self):
        # TODO clean!
        exp1 = rosparam.get_param('ueye_cam_nodelet_slave1/exposure')
        exp1 = float(exp1) - 0.3

        exp2 = rosparam.get_param('ueye_cam_nodelet_slave2/exposure')
        exp2 = float(exp2) - 0.3

        # FIXME: replace this part of two node exposure assignment introducing
        # the loop
        exposure_value = \
            dynamic_reconfigure.msg._DoubleParameter.DoubleParameter(name='exposure',
                                                                     value=exp1)
        service_request = dynamic_reconfigure.srv._Reconfigure.ReconfigureRequest()
        service_request.config.doubles.append(exposure_value)

        # TODO function.
        rospy.wait_for_service('/ueye_cam_nodelet_slave1/set_parameters')
        try:
            change_parameter = rospy.ServiceProxy(
                '/ueye_cam_nodelet_slave1/set_parameters', Reconfigure)
            response = change_parameter(service_request.config)
        except:
            # TODO recovery.
            rospy.logerr("service call failed")

        # ----------
        exposure_value = \
            dynamic_reconfigure.msg._DoubleParameter.DoubleParameter(name='exposure',
                                                                     value=exp2)
        service_request = dynamic_reconfigure.srv._Reconfigure.ReconfigureRequest()
        service_request.config.doubles.append(exposure_value)

        # TODO function.
        rospy.wait_for_service('/ueye_cam_nodelet_slave2/set_parameters')
        try:
            change_parameter = rospy.ServiceProxy(
                '/ueye_cam_nodelet_slave2/set_parameters', Reconfigure)
            response = change_parameter(service_request.config)
        except:
            # TODO recovery.
            rospy.logerr("service call failed")

        exposure_text = EXPOSURE_FORMAT.format(exp1, exp2)
        self.log_updated.emit(exposure_text)
        self.cameras_updated.emit(exposure_text)

        return (exp1, exp2)

    def reboot_computer(self):
        rospy.loginfo("reboot computer")
        subprocess.Popen(["sudo", "reboot"])

    def record_bag(self):
        """Start recording bag file. TODO(aql) more complete documentation.
        """
        self.launcher = subprocess.Popen(["roslaunch", "stereo_rig_gui",
                                          self.launch_bag_file])
        rospy.loginfo("runexp: Subprocess started")

    def stop_bag(self):
        """Stop recording bag file. TODO(aql) more complete documentation.
        """
        self.launcher.terminate()
        rospy.loginfo("runexp: Subprocess ended")

    def check_record_bag(self):
        if not self.started_recording:
            self.record_bag()
            self.started_recording = True
        else:
            self.stop_bag()
            self.started_recording = False
        self.diagnostic_updated.emit(self.started_recording, "recording")

    def handle_record_bag(self, req):
        self.check_record_bag()
        return []

    def change_parameter(self, req):
        # TODO separate?
        exp1 = rosparam.get_param('ueye_cam_nodelet_slave1/' + req.parameter)
        exp2 = rosparam.get_param('ueye_cam_nodelet_slave2/' + req.parameter)
        if req.increase:
            exp1 = float(exp1) + 0.3
            exp2 = float(exp2) + 0.3
        else:
            exp1 = float(exp1) - 0.3
            exp2 = float(exp2) - 0.3

        # FIXME: replace this part of two node exposure assignment introducing
        # the loop
        exposure_value = \
            dynamic_reconfigure.msg._DoubleParameter.DoubleParameter(name=req.parameter,
                                                                     value=exp1)
        service_request = dynamic_reconfigure.srv._Reconfigure.ReconfigureRequest()
        service_request.config.doubles.append(exposure_value)

        # TODO function.
        rospy.wait_for_service('/ueye_cam_nodelet_slave1/set_parameters')
        try:
            change_parameter = rospy.ServiceProxy(
                '/ueye_cam_nodelet_slave1/set_parameters', Reconfigure)
            response = change_parameter(service_request.config)
        except:
            # TODO recovery.
            rospy.logerr("service call failed")

       # ----------
        exposure_value = \
            dynamic_reconfigure.msg._DoubleParameter.DoubleParameter(name='exposure',
                                                                     value=exp2)
        service_request = dynamic_reconfigure.srv._Reconfigure.ReconfigureRequest()
        service_request.config.doubles.append(exposure_value)

        # TODO function.
        rospy.wait_for_service('/ueye_cam_nodelet_slave2/set_parameters')
        try:
            change_parameter = rospy.ServiceProxy(
                '/ueye_cam_nodelet_slave2/set_parameters', Reconfigure)
            response = change_parameter(service_request.config)
        except:
            # TODO recovery.
            rospy.logerr("service call failed")
        exposure_text = EXPOSURE_FORMAT.format(exp1, exp2)
        self.cameras_updated.emit(exposure_text)
        return True

    ### START Menu handler from manager.cpp in Aqua ###
    def menu(self, req):
        old_display_tags = self.display_tags
        self.display_tags = False
        self.question = ''
        ss = ""
        if req.title:
            ss = req.title
        i = 0
        for r in req.opts:
            ss = ss + str(i) + ':' + str(req.opts[i])
            if i != (len(req.opts) - 1):
                ss = ss + '\n'
            i = i + 1

        self.question = str(ss)
        self.tag_id = -1
        timedout = False
        done = False
        node = rospy.Rate(5)

        self.tag_sub = rospy.Subscriber(
            "stereo_rig/tags", Tags, self.handleTag, queue_size=1)
        while not rospy.is_shutdown() and (not timedout) and (not done) and (self.tag_id < 0 or self.tag_id > len(req.opts)):
            begin = rospy.get_rostime()
            self.question = str(ss)
            self.tag_id = -1

            while not rospy.is_shutdown() and self.tag_id == -1:
                node.sleep()
                if req.timeout > 0 and (rospy.get_rostime() - begin).secs > req.timeout:
                    timedout = True
                    break
            if timedout:
                self.question = "TOUT\n"+req.opts[req.default_choice]
                selection = req.default_choice
                done = True
            elif self.tag_id >= len(req.opts):
                self.question = "BAD"
            else:
                self.question = req.opts[self.tag_id]
                selection = self.tag_id
                done = True
            for i in range(0, 20):
                node.sleep()
        self.question = ''
        self.tag_id = -1
        self.tag_sub.unregister()
        self.display_tags = old_display_tags
        return selection

    '''
    def aski(req):
        self.question = req.name
        node = rospy.Rate(5)
        success = False
        done = False
        rospy.Subscriber("stereo_rig/tags", Tags, self.handleTag, queue_size=1)
        self.tag_id = -1
        while not rospy.is_shutdown() and  not done:
            self.question = req.name+"\n0=back"
            while not rospy.is_shutdown() and self.tag_id == -1:
                node.sleep()

    '''

    def handleTag(self, tags_msg):
        if len(tags_msg.tags) == 0:
            self.tag_id = -1
        else:
            self.tag_id = tags_msg.tags[0].id

    def handle_message(self, message):
        self.message = message.data
        self.message_time = rospy.get_rostime()

    def refresh_display(self, event):
        self.log_msg = ""
        if self.message:
            self.log_msg += self.message
            if (rospy.get_rostime - self.message_time).to_sec() > 1:
                self.message = ""
        if self.question:
            self.log_msg += self.question
        if self.tag_id >= 0:
            self.log_msg += "Tag: " + str(self.tag_id) + '\n'
        self.log_updated.emit(str(self.log_msg))
    ### END Menu handler from manager.cpp in Aqua ###


class StereoRigGuiProgram(QtWidgets.QDialog):
    """Stereo Rig GUI.
    """

    def __init__(self, app, launch_bag_file,
                 camera_name, camera_node):

        super(StereoRigGuiProgram, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('bluerov_gui_ros'),
                               'resource', 'bluerov.ui')
        loadUi(ui_file, self)
        # self.setWindowFlags(QtCore.Qt.CustomizeWindowHint)
        self.app = app
        self.current_image = None

        # Connect "add" button with a custom function (addInputTextToListbox)
        # self.addBtn.clicked.connect(self.addInputTextToListbox)

        # logger imformation
        self.log_msg = None

        # parameters.
        # Format float numbers.
        self.format_number = '{:.' + str(DECIMAL_PLACES) + 'f}'
        # launch file for start bagging.
        # Camera names (should match what is in header).
        self.camera_name = camera_name

        # Format of depth.
        # TODO setup of different measure unit.
        self.depth_in_feet = DEPTH_IN_FEET
        # Set measure unit for various data.
        if self.depth_in_feet:
            self.depth_label.setText(
                DATA_LABEL_FORMAT.format(DEPTH_STRING, FEET_STRING))
        else:
            self.depth_label.setText(
                DATA_LABEL_FORMAT.format(DEPTH_STRING, METER_STRING))

        # Temperature.
        self.temperature_label.setText(
            DATA_LABEL_FORMAT.format(TEMPERATURE_STRING, CELSIUS_STRING))

        # Pressure.
        self.pressure_label.setText(
            DATA_LABEL_FORMAT.format(PRESSURE_STRING, MBAR_STRING))

        # Exposure.
        self.exposure_label.setText(DATA_LABEL_FORMAT.format(
            EXPOSURE_STRING, MILLISECOND_STRING))

        # TODO create the menu.
        self.current_menu = "root"
        self.image_size = self.image_label.size()

        """
        left_camera_callback = functools.partial(
            self.read_image, camera_topic=os.path.join(camera_left_name, IMAGE_RAW_STRING))
        self._left_image_timer = QtCore.QTimer()
        self._left_image_timer.setInterval(1000)
        self._left_image_timer.timeout.connect(left_camera_callback)
        """
        self.ros_handler = RosHandler(
            launch_bag_file, camera_name, camera_node)

        self.ros_handler.image_updated.connect(self.change_image)
        self.ros_handler.diagnostic_updated.connect(
            self.change_diagnostic_value)
        self.ros_handler.log_updated.connect(self.change_log)
        self.ros_handler.cameras_updated.connect(self.change_exposure)
        self.ros_handler.message_initialization()

    def change_image(self, image, label):
        """Change image of the frame. TODO(aql) more complete documentation.
        """

        bytesPerLine = None
        qt_image = None

        # Resize image to fit in the UI label
        image = cv2.resize(
            image, (self.image_size.width(), self.image_size.height()))
        if image.ndim == 2:
            height, width = image.shape
            bytesPerLine = width
            qt_image = QtGui.QImage(image.data, width, height,
                                    bytesPerLine, QtGui.QImage.Format_Indexed8)
        elif image.ndim == 3:
            height, width, channel = image.shape
            bytesPerLine = 3 * width
            qt_image = QtGui.QImage(image.data, width, height,
                                    bytesPerLine, QtGui.QImage.Format_RGB888)
        else:
            rospy.logerr("Unsupported image format")

        self.image_label.setPixmap(QtGui.QPixmap.fromImage(qt_image))

    def change_diagnostic_value(self, value, label):
        """Change diagnostic value. TODO(aql) more complete documentation.
        """
        if label == "depth":
            if self.depth_in_feet:
                value *= METER_TO_FOOT
            self.depth_value_label.setText(self.format_number.format(value))
        elif label == "temperature":
            self.temperature_value_label.setText(
                self.format_number.format(value))
        elif label == "pressure":
            self.pressure_value_label.setText(self.format_number.format(value))
        elif label == "recording":
            self.recording_value_label.setText(str(bool(value)))
        elif label == "imu":
            msg_current_time = time.strftime('%H:%M:%S', time.localtime(value))
            self.imu_health_value.setText(msg_current_time)
        elif label == "sonar":
            msg_current_time = time.strftime('%H:%M:%S', time.localtime(value))
            self.sonar_health_value.setText(msg_current_time)

    def change_log(self, log_message):
        """Set message in the logger part.
        """
        self.message_value_label.setText(log_message)

    def change_exposure(self, exposure_text):
        """Set exposure values in GUI.
        """
        self.exposure_value_label.setText(exposure_text)


def shutdown_handler(*args):
    """Handler for the quit signal."""
    rospy.signal_shutdown("Received a termination signal.")
    QtWidgets.QApplication.quit()


def starting_up(argv):
    """Booting the GUI."""
    global CAMERA_NAME
    global LAUNCH_BAG_FILE

    # Initialization of Qt app.
    app = QtWidgets.QApplication(argv)

    # Read parameters.
    # TODO(aql) topics and functions from file.
    launch_bag_file = rospy.get_param('launch_bag_file', LAUNCH_BAG_FILE)
    camera_name = rospy.get_param('camera_name', CAMERA_NAME)
    camera_node = rospy.get_param('camera_node', CAMERA_NODE)

    # initialize actual window.
    prog = StereoRigGuiProgram(app, launch_bag_file,
                               camera_name, camera_node,
                               )
    prog.show()

    # Handling of application quit.
    rospy.on_shutdown(shutdown_handler)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)

    sys.exit(app.exec_())


if __name__ == '__main__':
    starting_up(sys.argv)
