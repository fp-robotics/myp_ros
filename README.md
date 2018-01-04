# ROS Package for myP 1.3


### Overview
This ROS Package contains the message files needed for reading the ROS topics and the service files needed for executing myP functions and scripts via ROS. Also, some examples are available.


### Building
If you do not already have a catkin workspace set up, please set one up according to this tutorial: http://wiki.ros.org/catkin/Tutorials/create_a_workspace.

Clone the *myp_ros* repository in the source folder of your catkin workspace and build it by calling either *catkin_make* or *catkin build*. To use this package in a terminal shell, you must first run
```sh
$ source PATH_TO_CATKIN_WS/devel/setup.bash
```
where PATH_TO_CATKIN_WS is the location of your catkin workspace. To have this package available in all open terminal shells, you may add this line to your *.bashrc* file by running
```sh
$ echo "source PATH_TO_CATKIN_WS/devel/setup.bash" >> ~/.bashrc
```


### Runtime
On startup, the PRob will begin a roscore as well as a node which provides both the ROS communication and the browser based interface. Only one of these communication channels can be used to send commands to the robot at a time however after connecting over ROS, all outgoing information continues to be published their corresponding topics even after switching connections to the browser.

To use the same ROS Master as the PRob you must first have your machine appropriately set up for ROS network communication according to http://wiki.ros.org/ROS/NetworkSetup. You must then export the correct master URI. Open up a terminal and enter:
```sh
$ export ROS_MASTER_URI=http://<PROB_IP>:11311
```
where <PROB_IP> is the IP address of your PRob. In some system, you might need to manually set the ROS_IP using
```sh
$ export ROS_IP=<YOUR_IP>
```

To see your IP you can use the command: `hostname -I`

To test this connection, make sure there is no roscore running locally on your machine and check for available ROS topics and services by running
```sh
$ rostopic list
$ rosservice list
```

You should see a list of topics and services. To open up all channels of communication over ROS, use the ROS service 'connect' provided by the PRob.


### myP ROS Messages
myP publishes messages to the following topics:
- Joint Information Topics
  - /joint_positions
  - /joint_velocities
  - /joint_currents
  - /joint_states
- Tool Center Point Information Topics
  - /pose
  - /pose_alternative
  - /transform
- Sensor Information Topics
  - /sensors
  - /digital_inputs
  - /digital_outputs


### myP ROS Services
myP provides a lot of services, basically one for each script function available via the myP GUI. In addition to those services, myP also provides a couple of special myP and P-Rob control services:
- Script Function Services
  - /move_joint
  - /move_to_pose
  - /open_gripper
  - ... (lots more, check out the *myP Script Functions Manual* for a complete list of functions)
- Special myP Initialization Services
  - /connect
  - /calibrate
  - /disconnect
  - /get_status
  - /initialize_application
  - /finalize_application
- Special P-Rob Control Services
  - /pause
  - /resume
  - /stop
  - /recover
  - /release
  - /hold

All of these services have been designed to use one and the same service file (Generic_myP_Service.srv), which takes a JSON string as its input argument. In order to execute a function which in Python would look like
```python
func(*args, **kwargs)
```
it must be converted to the following JSON string for using it with myP ROS services:
```python
"{'func': func, 'args': args, 'kwarg1': value1, 'kwarg2': value2, ...}"
```


# Examples and Extras

### Joystick controller
The node joy_control.py subscribes to the joy topic to allow a user to use a joystick to move the end effector in steps. Inorder to use this functionality, set up your machine to run a joy_node according to the following tutorial http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick. Remember to export the MyP ROS_MASTER_URI in all your peripheral nodes.

The mapping used for this controller is that of a Sony PS3 Controller, you may need to change the mappings in the node if your controller does not follow the same pattern. The loop rate in the node can be adjusted to you choosing. The reference position updates much faster than the movement commands sent, so with a slower rate you have more time to adjust the desired position before the robot is told to move there.
