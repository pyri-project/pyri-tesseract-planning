from RobotRaconteurCompanion.Util.GeometryUtil import GeometryUtil
from RobotRaconteurCompanion.Util.RobotUtil import RobotUtil
from pyri.plugins.sandbox_functions import PyriSandboxFunctionsPluginFactory
from pyri.sandbox_context import PyriSandboxContext
import numpy as np
import time
import general_robotics_toolbox as rox

import RobotRaconteur as RR

def _get_active_robot_name():
    # TODO: verify robot exists
    if "active_robot" in PyriSandboxContext.context_vars:
        return PyriSandboxContext.context_vars["active_robot"]
    else:
        return "robot"

def tesseract_get_robot_position():
    """
    Get the position of the active virtual twin robot
    """

    robot_name = _get_active_robot_name()

    device_manager = PyriSandboxContext.device_manager
    planner_service = device_manager.get_device_client("tesseract_planner", 1)
    v = planner_service.getf_robot_joint_values(robot_name)
    v2 = np.rad2deg(v)
    return v2

def tesseract_get_robot_position():
    """
    Get the position of the active virtual twin robot
    """

    robot_name = _get_active_robot_name()

    device_manager = PyriSandboxContext.device_manager
    planner_service = device_manager.get_device_client("tesseract_planner", 1)
    v = planner_service.getf_robot_joint_values(robot_name)
    v2 = np.rad2deg(v)
    return v2

def tesseract_set_robot_position(position):
    """
    Set the position of the active virtual twin robot
    """

    robot_name = _get_active_robot_name()

    device_manager = PyriSandboxContext.device_manager
    planner_service = device_manager.get_device_client("tesseract_planner", 1)
    v = np.deg2rad(position)
    planner_service.setf_robot_joint_values(robot_name,v)

def tesseract_sync_robot():
    """
    Sync virtual twin robot to real robot
    """

    robot_name = _get_active_robot_name()

    device_manager = PyriSandboxContext.device_manager
    planner_service = device_manager.get_device_client("tesseract_planner", 1)
    
    robot = device_manager.get_device_client(robot_name,1)
    robot_state, _ = robot.robot_state.PeekInValue()
    v = robot_state.joint_position

    planner_service.setf_robot_joint_values(robot_name,v)

def tesseract_plan_freespace_joint_target(target_joint_position, speed_perc):
    """
    Plan collision-free freespace move with a joint target for
    active robot
    """

    robot_name = _get_active_robot_name()

    device_manager = PyriSandboxContext.device_manager
    planner_service = device_manager.get_device_client("tesseract_planner", 1)
    robot = device_manager.get_device_client(robot_name,1)
    robot_info = robot.robot_info

    joint_names = []
    k = robot_info.chains[0]
    for j in k.joint_numbers:
        joint_names.append(robot_info.joint_info[j].joint_identifier.name)

    joint_v = np.deg2rad(target_joint_position)

    res_gen = planner_service.plan_freespace_move(robot_name, joint_v, speed_perc)

    res = None

    while True:
        try:
            res = res_gen.Next()
        except RR.StopIterationException:
            break

    traj = res.trajectory
    traj.joint_names = joint_names

    return traj


def _get_sandbox_functions():
    return {
        "tesseract_get_robot_position": tesseract_get_robot_position,
        "tesseract_set_robot_position": tesseract_set_robot_position,        
        "tesseract_sync_robot": tesseract_sync_robot,
        "tesseract_plan_freespace_joint_target": tesseract_plan_freespace_joint_target
    }

class RoboticsSandboxFunctionsPluginFactory(PyriSandboxFunctionsPluginFactory):
    def get_plugin_name(self):
        return "pyri-tesseract-planner"

    def get_sandbox_function_names(self):
        return list(_get_sandbox_functions().keys())

    def get_sandbox_functions(self):
        return _get_sandbox_functions()


def get_sandbox_functions_factory():
    return RoboticsSandboxFunctionsPluginFactory()

