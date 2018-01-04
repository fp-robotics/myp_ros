#!/usr/bin/env python
"""
A node that uses the joystick to control the end effector:

L-LR: 		msg.axes[0]
L-UD: 		msg.axes[1]
R-LR: 		msg.axes[2]
R-UD: 		msg.axes[3]

triangle: 	buttons[12]
x:          buttons[14]
"""
import rospy

from copy import deepcopy
from rospy import ServiceException
from rospy.exceptions import ROSInterruptException

from sensor_msgs.msg import Joy

from myp_ros.scripts.myp_ros_functions import move_joint, move_tool, play_path, open_gripper, close_gripper
from myp_ros.srv import Generic_myP_Service


class JoystickController:
    def __init__(self):
        self.reset_pose_args = [189.0, 0.0, 1189.5,  [0.0, 0.0, 0.0], 80, 80, False, False, '']
        move_tool(*self.reset_pose_args)

        self.button_pressed_start = 0
        self.button_pressed_select = 0
        self.button_pressed_triangle = 0
        self.button_pressed_x = 0
        self.button_pressed_square = 0
        self.button_pressed_circle = 0

        rospy.init_node('joystick_controller', anonymous=True)

        self.scale = 1.0

        move_joint(actuator_ids=['1', '2', '3', '4', '5', '6'], position=0, velocity=80, acceleration=80)

        self.update_pos = deepcopy(self.reset_pose_args)
        self.last_good = deepcopy(self.reset_pose_args)

        rospy.Subscriber('joy', Joy, self.callback)

        rospy.loginfo("started joystick_controller")
        rospy.loginfo("\nUse the analog joysticks to control the robot's end effector."
                      "\nUse the left joystick to control x-y and the right to control z."
                      "\nPress Triangle (PS3) to increase the joystick scale."
                      "\nPress Circle   (PS3) to decrease the joystick scale.")

    # callback for incoming raw joystick data
    def callback(self, msg):
        # updating the x-y-z coordinates according to message inputs
        self.update_pos[0] += self.scale * msg.axes[0]
        self.update_pos[1] -= self.scale * msg.axes[1]
        self.update_pos[2] += self.scale * msg.axes[3]

        # run function according to button pressed
        if self.button_pressed_select != msg.buttons[0] and self.button_pressed_select == 0:
            rospy.loginfo("pressing 'select' button")
            play_path(path_name='path_name')

        if self.button_pressed_start != msg.buttons[3] and self.button_pressed_start == 0:
            rospy.loginfo("pressing 'start' button")
            self.update_pos = deepcopy(self.reset_pose_args)

        if self.button_pressed_triangle != msg.buttons[12] and self.button_pressed_triangle == 0:
            rospy.loginfo("pressing 'triangle' button")
            self.scale_up()

        if self.button_pressed_circle != msg.buttons[13] and self.button_pressed_circle == 0:
            rospy.loginfo("pressing 'circle' button")
            close_gripper()

        if self.button_pressed_x != msg.buttons[14] and self.button_pressed_x == 0:
            rospy.loginfo("pressing 'x' button")
            self.scale_down()

        if self.button_pressed_square != msg.buttons[15] and self.button_pressed_square == 0:
            rospy.loginfo("pressing 'square' button")
            open_gripper()

        # update button state
        self.button_pressed_select = msg.buttons[0]
        self.button_pressed_start = msg.buttons[3]
        self.button_pressed_triangle = msg.buttons[12]
        self.button_pressed_circle = msg.buttons[13]
        self.button_pressed_x = msg.buttons[14]
        self.button_pressed_square = msg.buttons[15]

    def scale_up(self):
        self.scale *= 1.1
        rospy.loginfo('new scale factor: ' + str(self.scale))

    def scale_down(self):
        self.scale /= 1.1
        rospy.loginfo('new scale factor: ' + str(self.scale))

    def reset_tool(self):
        rospy.loginfo('resetting tool')
        move_tool(*self.reset_pose_args)

    def move_tool_service(self):
        try:
            move_tool(*self.update_pos)
            self.last_good = deepcopy(self.update_pos)
        except Exception as error:
            self.update_pos = deepcopy(self.last_good)
            print(error)

    """
    Define a function to return to neutral position to be called on shutdown.
    In shutdown ROS services cannot provide the return value so we may use
    the ServiceException raised by this as the trigger for service completion.
    """
    def move_back(self):
        try:
            move_joint(actuator_ids=['1', '2', '3', '4', '5', '6'], position=0, velocity=80, acceleration=80)
        except ServiceException:
            print("Done. Goodbye!")


# publish scaled and filtered data at a rate of 150Hz
if __name__ == '__main__':
    jc = JoystickController()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        try:
            jc.move_tool_service()
            rate.sleep()

        except ROSInterruptException as error:
            rospy.loginfo("Movement interrupted by shutdown.")
            jc.move_back()
