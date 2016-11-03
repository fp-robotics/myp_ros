# Running ROS with MyP
### Building
If you do not already have a catkin workspace set up please set one up according to this tutorial http://wiki.ros.org/catkin/Tutorials/create_a_workspace.

Place the *myp_ros* directory in the source folder of your catkin workspace and buildby calling either *catkin_make* or *catkin build* . To use this package in a terminal shell, you must first run:
```sh
$ source PATH_TO_CATKIN_WS/devel/setup.bash
```
Where PATH_TO_CATKIN_WS is the location of your catkin workspace. To have this package available in every terminal shell opneded, you may add this line to your .bashrc via:

```sh
$ echo "source PATH_TO_CATKIN_WS/devel/setup.bash" >> ~/.bashrc
```
### Runtime
On startup, the PRob will begin a roscore as well as a node which provides both the ROS communication and the browser based interface. Only one of these communication channels can be used to send commands to the robot at a time however after connecting over ROS, all outgoing information continues to be published their corresponding topics even after switching connections to the browser.

To use the same ROS Master as the PRob you must first have your machine appropriately set up for ROS network communication according to http://wiki.ros.org/ROS/NetworkSetup. You must then export the correct master URI. Open up a terminal and enter:

```sh
$ export ROS_MASTER_URI=http://<PROB_IP>:11311
```

Where <PROB_IP> is the IP address of your PRob. To test this connection, make sure there is no roscore running locally on your machine and check for available services with:

```sh
$ rosservice list
```

You should see a list of services. To open up all channels of communication over ROS, use the ROS service 'connect' provided by the PRob.

The first parameter must be a string, either 'real' or 'no_robot' depending on whether you want, the second is a string detailing your robot model, e.g 'PRob2R', the third parameter, also a string, determines the calibration type, which should always be 'default'.

To see each of the services provided and their parameters, look under in the *myp_ros/srv* directory. To see a basic example of each service, as well as subscription to each topic, see the node *example.py* located in *myp_ros/scripts*

# Extras
### Joystick controller
The node joy_control.py subscribes to the joy topic to allow a user to use a joystick to move the end effector in steps. Inorder to use this functionality, set up your machine to run a joy_node according to the following tutorial http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick. Remember to export the MyP ROS_MASTER_URI in all your peripheral nodes.

The mapping used for this controller is that of a Sony PS3 Controller, you may need to change the mappings in the node if your controller does not follow the same pattern. The loop rate in the node can be adjusted to you choosing. The reference position updates much faster than the movement commands sent, so with a slower rate you have more time to adjust the desired position before the robot is told to move there.
