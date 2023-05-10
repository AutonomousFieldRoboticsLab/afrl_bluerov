# BlueROV Tele-operation How-to

This document describes how-to teleoperate BlueROV using the tether
and a joystick. Note that this instructions are only relevant to the
`x_mode_joystick_control` branch of the `afrl_bluerov` repository.


## Configuring the network

### Setting up IP Address

The blueROV tether interface is in manual mode and has an IP address
set to `192.168.100.1` with hostname `brov`. The remote computer should
be cofigured with an IP address in the the same subnet, for example -
`192.168.100.2`.

### Edit hosts file

Edit `/etc/hosts` file in remote-computer and add two entries for the
blueROV and the remote computer:

```
192.168.100.1 brov
192.168.100.x <remote-computer-name>
```

Similarly, edit `/etc/hosts` file in remote computer.

If everything is setup correctly, you should be able to ping the robot
with `ping brov` and vice versa.


## Running nodes in host computer

The teleop will work only if all required nodes are running inside the
robot. Check if `rostopic list` has a bunch of topics with prefix -
`/bluerov/` and `/mavros/`. If not, go into appropriate workspace and
spawn the nodes with -

```
roslaunch bluerov_bringup bluerov_bringup.launch
```


## Configuring and testing the Logitech F719 Joystick in X-mode

- Connect the Joystick dongle to the remote computer.
- If a green led is turned on, press the __mode__ button
  to turn it off.
- Set the input-mode slider to `x`, NOT `d`. This configures
  the joystick to use X-input mode which let's us use the analog
  triggers as another set of analog axis.
- To check if the joystick is working start `jstest /dev/input/js0`
  into a terminal and see if the values are changing with key-press.


## Running nodes in remote computer

- Open a terminal.
- Setup two environment variables with:

```
export ROS_HOSTNAME=<remote_computer_name>
export ROS_MASTER_URI=http://brov:11311/
```

- Go into catkin workspace and activate the developer environment.
- Start the teleop node with `roslaunch bluerov_control_ros bluerov_teleop.launch`. 
  This should spawn an `rviz` instance showing the camera feed.
  The node also starts a bag recorder that dumps timestamped bagfile
  into the home directory with name prefix - `bluerov_teleop`.


## Controlling the Robot with Joystick

- Arm the motors by pressing the __Start__ button. Wait a few seconds.
- Accelerate forward with __Right Trigger(RT)__ button. That's the
  analog button you can press with your right index finger.
- Accelerate backward with __Left Trigger(LT)__ button.
- Yaw is controlled by x-axis of __Left Thumb Analog Stick__.
- Left-right lateral movement is controlled by x-axis of
  __Right Thumb Analog Stick__. Ascend-descend vertical
  movement is controlled by y-axis of the same stick.
- Roll is controlled by __Left Shoulder Button (LB)__ and
  __Right Shoulder Button (RB)__.
- Light can be turned on and off with up and down buttons of
  the __Directional-PAD__.
- __Back__ button disarms the motors.

