# ROS Package for myP 1.3

##### &copy; F&amp;P Robotics AG, Switzerland (http://www.fp-robotics.com)

### Overview

This ROS Package contains everything you need to communicate and control a P-Rob personal robot via ROS. This package provides you with ROS message files needed for communication between myP (pre-installed on P-Rob), ROS topics for observing myP and P-Rob, as well as ROS service files needed for executing myP functions and applications via ROS. Finally, this package also provides you with a couple of usage examples.

### Building Package

If you do not already have a catkin workspace set up, please set one up according to this tutorial: http://wiki.ros.org/catkin/Tutorials/create_a_workspace. Clone the **myp_ros** repository in the source folder of your catkin workspace and build it by calling either **catkin_make** or **catkin build**. To use this package in a terminal shell, you must first run

```sh
$ source <CATKIN_WS_PATH>/devel/setup.bash
```

where **<CATKIN_WS_PATH>** is the location of your catkin workspace. To have this package available in all open terminal shells, you may add this line to your **.bashrc** file by running

```sh
$ echo "source <CATKIN_WS_PATH>/devel/setup.bash" >> ~/.bashrc
```

### Runtime &amp; Control

When starting up P-Rob, myP runs together with a ROS core and a ROS node. myP supports both the standard control via its browser GUI interface, as well as control via ROS services. Only one of these control channels can be actively used to send commands to P-Rob at any time, meaning that connecting myP via ROS prevents the user to interfere into running applications using the myP browser GUI. The graphical interface can still be used for e.g. state observation though. All information leaving P-Rob continues to be broadcast and published to their corresponding ROS topics, even if no browser is watching or observing P-Rob. For more information about the GUI of myP, please refer to the **myP User Manual** provided by F&amp;P Robotics.

To use the same ROS Master as P-Rob you must first have your machine appropriately set up for ROS network communication according to http://wiki.ros.org/ROS/NetworkSetup. You must then export the correct master URI. In some systems, you might also need to manually set the **ROS_IP**. Open up a terminal and enter:

```sh
$ export ROS_MASTER_URI=http://<PROB_IP>:11311
$ export ROS_IP=<YOUR_IP>
```

where **<PROB_IP>** and **<YOUR_IP>** are the IP addresses of your P-Rob and your control computer, respectively. In unix systems, you can checkout the IP address of your system by executing

```sh
$ hostname -I
```

To test this connection, make sure there is no ROS core running locally on your machine and check for available ROS topics and services by running

```sh
$ rostopic list
$ rosservice list
```

You should now see a list of available topics and services. To open up all channels of communication over ROS, use the ROS service **/<ROBOT_NAME>/connect** provided by P-Rob, where **<ROBOT_NAME>** of course must be replaced by the name of your robot, e.g. **PRob2R**.

### myP ROS Messages

myP publishes messages to the following topics:
- Joint Information Topics
  - /<ROBOT_NAME>/joint_positions
  - /<ROBOT_NAME>/joint_velocities
  - /<ROBOT_NAME>/joint_currents
  - /<ROBOT_NAME>/joint_states
- Tool Center Point Information Topics
  - /<ROBOT_NAME>/pose
  - /<ROBOT_NAME>/pose_alternative
  - /<ROBOT_NAME>/transform
- Sensor Information Topics
  - /<ROBOT_NAME>/sensors
  - /<ROBOT_NAME>/digital_inputs
  - /<ROBOT_NAME>/digital_outputs

### myP ROS Services

myP provides a lot of services, basically one for each script function available via the myP GUI. In addition to those services, myP also provides a couple of special myP and P-Rob control services:
- Script Function Services
  - /<ROBOT_NAME>/move_joint
  - /<ROBOT_NAME>/move_to_pose
  - /<ROBOT_NAME>/open_gripper
  - ... and lots more, check out the **myP Script Functions Manual** by F&amp;P Robotics for a complete list of functions
- Special myP Initialization Services
  - /<ROBOT_NAME>/connect
  - /<ROBOT_NAME>/calibrate
  - /<ROBOT_NAME>/disconnect
  - /<ROBOT_NAME>/get_status
  - /<ROBOT_NAME>/initialize_application
  - /<ROBOT_NAME>/finalize_application
- Special P-Rob Control Services
  - /<ROBOT_NAME>/pause
  - /<ROBOT_NAME>/resume
  - /<ROBOT_NAME>/stop
  - /<ROBOT_NAME>/recover
  - /<ROBOT_NAME>/release
  - /<ROBOT_NAME>/hold

All of these services have been designed to use one and the same service file (Generic_myP_Service.srv), which takes a string as its input argument. This string supports three different formats, you may use whichever you like best or fits your purpose best:
- (JSON format) ``` "{'arg_1': value_1, 'arg_2': value_2, 'arg_3': value_3, ...}" ```
- (JSON format) ``` "[value_1, value_2, value_3, ...]" ```
- (Pythonic format) ``` "value_1 value_2 arg3=value3 arg4=value4" ```

For functions without input arguments, especially the special initialization and control services mentioned above, you can use the empty input string `""`.

# Examples and Extras

### Motion control using ROS services

Here's a small terminal example to check if the robot is not already initialized (status is "none"), initializing a myP application and executing a bunch of script functions.

```sh
$ rosservice call /PRob2R/get_status ""
"none"
# If the status is "none", the robot is not initialized and you may connect.
# Otherwise, the robot must be disconnected via myP GUI first.
$ rosservice call /PRob2R/connect ""
$ rosservice call /PRob2R/calibrate ""
# Now the robot is ready to execute myP applications.
# Let's use an application with 'move_joint' command with different input formats:
$ rosservice call /PRob2R/initialize_application ""
$ rosservice call /PRob2R/move_joint "'{\"actuator_ids\": [5, 6], \"position\": 90, \"velocity\": 20}'"
$ rosservice call /PRob2R/open_gripper ""
$ rosservice call /PRob2R/move_joint "'[[5, 6], -90, 20]'"
$ rosservice call /PRob2R/close_gripper ""
$ rosservice call /PRob2R/move_joint "'[5, 6] 0 velocity=20'"
$ rosservice call /PRob2R/finalize_application ""
# Now the first application is done, let's move the robot manually ...
$ rosservice call /PRob2R/release ""
# ... and go back to "ready" status for further applications.
$ rosservice call /PRob2R/hold ""
```

### Joystick controller

The node joy_control.py subscribes to the joy topic to allow a user to use a joystick to move the end effector in steps. Inorder to use this functionality, set up your machine to run a joy_node according to the following tutorial http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick. Remember to export the myP ROS_MASTER_URI in all your peripheral nodes.

The mapping used for this controller is that of a Sony PS3 Controller, you may need to change the mappings in the node if your controller does not follow the same pattern. The loop rate in the node can be adjusted to you choosing. The reference position updates much faster than the movement commands sent, so with a slower rate you have more time to adjust the desired position before the robot is told to move there.
