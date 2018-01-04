#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This file contains functions to make it easier to use the available myP ROS services. After importing all functions from
this file, you can basically copy-paste a script running in the myP GUI into a standard Python file and execute those
functions via ROS instead of the GUI.

Usage example:

> from myp_ros.examples.myp_ros_functions import *
> connect()
> calibrate()
> initialize_application()
> for joint in range(6):
>     move_joint(joint,  10, velocity=50, acceleration=100)
>     move_joint(joint, -10, velocity=50, acceleration=100)
>     move_joint(joint,   0, velocity=50, acceleration=100)
> finalize_application()
"""
import inspect
import json
import rospy

from copy import deepcopy

from myp_ros.srv import Generic_myP_Service


# Here is a list of myP functions available as ROS services:
_script_function_services = ['move_joint', 'move_tool', 'move_to_pose', 'play_path', 'run_simple_path',
                             'calculate_advanced_path', 'run_advanced_path', 'wait_for_motor', 'stop_motion',
                             'open_gripper', 'close_gripper', 'read_gripper_angle', 'recognize_object',
                             'move_linear_axis', 'read_tcp_pose', 'get_grid_points', 'forward_kinematics',
                             'inverse_kinematics', 'wait', 'run_script', 'start_parallel_script', 'stop_application',
                             'pause_application', 'stop_and_release', 'dialog_text', 'dialog_yes_no', 'say',
                             'raise_error', 'raise_warning', 'log_message', 'print_version_info', 'set_shared_variable',
                             'get_shared_variable', 'remove_shared_variable', 'set_global_variable',
                             'get_global_variable', 'remove_global_variable', 'add_state_variable',
                             'set_state_variable', 'get_state_variable', 'remove_state_variable',
                             'reset_state_variable', 'set_state_variable_default', 'read_sensor_data',
                             'send_sensor_instruction', 'get_sensors', 'add_sensors', 'remove_sensors',
                             'read_digital_inputs', 'read_digital_outputs', 'write_digital_outputs',
                             'read_actuator_position', 'read_actuator_velocity', 'read_actuator_current',
                             'get_velocity', 'calibrate_joint', 'calibrate_gripper', 'calibrate_linear_axis',
                             'enable_workspace_boundaries', 'disable_workspace_boundaries', 'get_forbidden_zones',
                             'add_forbidden_zone', 'remove_forbidden_zone', 'open_camera', 'close_camera',
                             'start_camera_service', 'stop_camera_service', 'reset_vision_function', 'set_camera_mode',
                             'read_camera_data', 'transform_camera_data', 'set_plane_to_intersect',
                             'set_camera_autofocus', 'set_camera_whitebalance', 'get_points_in_toolframe',
                             'transform_vector_from_camera_to_base_coordinates', 'display_camera_point_in_base_frame',
                             'get_points_and_normals', 'get_collision_sensitivity', 'set_collision_sensitivity',
                             'enable_collision_detection', 'disable_collision_detection', 'get_current_deviation',
                             'set_current_deviation', 'get_task', 'play_task', 'load_task', 'save_task', 'play_skill',
                             'get_lesson', 'edit_lesson', 'create_lesson', 'delete_lesson', 'start_training',
                             'record_sample', 'save_and_train', 'calibrate_sensor', 'save_sensor_calibration',
                             'load_sensor_calibration', 'disable_log', 'enable_log', 'get_log_status', 'upgrade_config',
                             'move_into_mechanical_stop', 'move_into_limit_switch', 'is_limit_switch_active',
                             'start_myp_version', 'finalize_calibration']

# Here is a list of control functions available as ROS services:
_control_function_services = ['connect', 'calibrate', 'disconnect', 'get_status', 'pause', 'resume',
                              'stop', 'recover', 'release', 'hold', 'initialize_application',
                              'finalize_application', 'wait_for_info_from_myP', 'send_info_to_myP']


class ROSRobotError(Exception):
    """
    This helper class is used to handle myP errors after passing through ROS services.
    """
    def __init__(self, code, message, title, type):
        self.code = code
        self.message = message
        self.title = title
        self.type = type


_builtin_exceptions = dict()
for key, value in locals()['__builtins__'].items():
    if inspect.isclass(value) and issubclass(value, Exception):
        _builtin_exceptions[key] = value


def _create_function(function_name, robot_name=''):
    """
    This function recreates the Python function belonging to a myP ROS service. After using this function, you may use
    func(*args, **kwargs) again, instead of sending the JSON string "{'func': func, 'args': args, 'kwarg1': ...}".
    Also, this function contains some error handling, so errors raised by myP via ROS services are raised here instead.
    """
    service_name = robot_name + '/' + function_name if robot_name else function_name

    def function_template(*args, **kwargs):
        # create function call string to send
        input_dict = deepcopy(kwargs)
        input_dict.update({'func': function_name, 'args': args})
        input_string = json.dumps(input_dict)
        # create service proxy to ROS service
        service_proxy = rospy.ServiceProxy(service_name, Generic_myP_Service)
        # execute function via service proxy
        try:
            response = service_proxy(input_string)
        except rospy.ServiceException as error:
            print("ROS Service '" + str(service_name) + "' did not process request: " + str(error))
            raise error
        # extract values from output -> the output format is [return_value, error_string]
        output_string = response.output
        output_value, error = json.loads(output_string)
        if error:
            color_red = '\033[91m'
            color_default = '\033[0m'
            if type(error) is dict:
                if 'code' in error.keys():
                    print(color_red + 'Error (' + error['code'] + '): '
                          + error['title'] + ', ' + error['message'] + color_default)
                    raise ROSRobotError(**error)
                elif error['type'] in _builtin_exceptions.keys():
                    print(color_red + error['type'] + ': ' + error['message'] + color_default)
                    raise _builtin_exceptions[error['type']](error['message'])
                else:
                    print(color_red + str(error) + color_default)
            else:
                print(color_red + str(error) + color_default)
        # return output
        return output_value

    return function_template


# create a ROS service proxy for all myP script functions and add them to the local namespace of this file
for _function in _script_function_services:
    locals().update({_function: _create_function(_function)})


# create a ROS service proxy for all special control functions available in addition to the script functions
for _function in _control_function_services:
    locals().update({_function: _create_function(_function)})
