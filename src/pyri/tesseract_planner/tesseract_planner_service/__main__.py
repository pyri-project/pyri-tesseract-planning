import sys
import RobotRaconteur as RR

RRN=RR.RobotRaconteurNode.s
import numpy as np
import argparse
import RobotRaconteurCompanion as RRC
from pyri.device_manager_client import DeviceManagerClient
import importlib.resources as resources
from RobotRaconteurCompanion.Util.InfoFileLoader import InfoFileLoader
from RobotRaconteurCompanion.Util.AttributesUtil import AttributesUtil
from RobotRaconteurCompanion.Util.RobotUtil import RobotUtil
from RobotRaconteurCompanion.Util.GeometryUtil import GeometryUtil

import general_robotics_toolbox as rox
import copy
import time
import threading
import os
import re
import traceback

import importlib.metadata

from pyri.util.service_setup import PyriServiceNodeSetup

from tesseract_robotics.tesseract_common import FilesystemPath, Isometry3d, Translation3d, Quaterniond, \
    ManipulatorInfo
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_scene_graph import SimpleResourceLocator, SimpleResourceLocatorFn
from tesseract_robotics.tesseract_command_language import StateWaypoint, Waypoint, MoveInstruction, \
    Instruction, CompositeInstruction, PlanInstruction, JointWaypoint, MoveInstructionType_FREESPACE, \
    PlanInstructionType_FREESPACE, PlanInstructionType_START, flatten, isMoveInstruction, isStateWaypoint
from tesseract_robotics.tesseract_process_managers import ProcessPlanningServer, ProcessPlanningRequest, \
    FREESPACE_PLANNER_NAME

from tesseract_robotics.tesseract_time_parameterization import IterativeSplineParameterization

from tesseract_robotics_viewer import TesseractViewer

def _get_robot_names(t_env):
    manip_manager = t_env.getManipulatorManager()

    group_names = list(manip_manager.getGroupNames())
    robot_names = []
    for group_name in group_names:
        if manip_manager.hasChainGroup(group_name):
            robot_names.append(group_name)
    return robot_names

def _get_robot_joint_names(t_env, robot_name):
    manip_manager = t_env.getManipulatorManager()

    assert manip_manager.hasChainGroup(robot_name), f"Planner error: \"{robot_name}\" is not an SRDF chain group, cannot find joint names for planning"

    chain_groups = manip_manager.getChainGroup(robot_name)

    # TODO: return of getChainGroup() is a vector_string_pair. This should just be string_pair. Likely to change in future?
    assert len(chain_groups) == 1, "Internal tesseract service error in _get_robot_joint_names"
    
    chain_group = chain_groups[0]

    base_link, tip_link = chain_group

    all_link_names = list(t_env.getLinkNames())
    assert base_link in all_link_names, "Joint \"{base_link}\" for robot \"{robot_name}\" not found"
    assert tip_link in all_link_names, "Joint \"{tip_link}\" for robot \"{robot_name}\" not found"

    scene_graph = t_env.getSceneGraph()

    graph_path = scene_graph.getShortestPath(base_link, tip_link)

    all_joints = list(graph_path[1])   
    joint_names=[]

    active_joint_names = t_env.getActiveJointNames()

    for j_name in all_joints:
        if j_name in active_joint_names:
            joint_names.append(j_name)
   
    return joint_names    

class TesseractPlannerService_impl:
    def __init__(self, device_info, t_env, viewer, node: RR.RobotRaconteurNode = None):
        if node is None:
            self._node = RR.RobotRaconteurNode.s
        else:
            self._node = node
        self.device_info = device_info

        self.service_path = None
        self.ctx = None

        self._geom_util = GeometryUtil(node=self._node)
        self._robot_util = RobotUtil(node=self._node)

        self._trajectory_type = self._node.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectory")
        self._trajectory_wp_type = self._node.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectoryWaypoint")

        self._t_env = t_env
        self._viewer = viewer

        self._lock = threading.Lock()

        robot_names = _get_robot_names(t_env)
        for r in robot_names:
            _get_robot_joint_names(t_env,r)

        self._planning_server = ProcessPlanningServer(self._t_env, 1)
        self._planning_server.loadDefaultProcessPlanners()

        self._spline_param = IterativeSplineParameterization()


    def RRServiceObjectInit(self, ctx, service_path):
        self.service_path = service_path
        self.ctx = ctx

    def getf_robot_names(self):
        with self._lock:
            return _get_robot_names(self._t_env)

    def getf_robot_joint_names(self, robot_name):
        with self._lock:
            return _get_robot_joint_names(self._t_env, robot_name)

    def getf_robot_joint_values(self, robot_name):
        with self._lock:
            joint_names =  _get_robot_joint_names(self._t_env, robot_name)
            ret = self._t_env.getCurrentJointValues(joint_names).flatten()
            return ret

    def setf_robot_joint_values(self, robot_name, joint_value):
        with self._lock:
            joint_names =  _get_robot_joint_names(self._t_env, robot_name)
            assert len(joint_names) == len(joint_value), "joint_value length must equal number of active joints in robot"

            self._t_env.setState(joint_names, joint_value)

            self._update_viewer()

    def plan_freespace_move(self, robot_name, target_joint_value, speed_perc):
        with self._lock:
            assert speed_perc > 0 and speed_perc <= 100, "speed_perc must be between 1 and 100"
            joint_names = list(_get_robot_joint_names(self._t_env, robot_name))
            initial_joint_value = self._t_env.getCurrentJointValues(joint_names).flatten()
            assert len(target_joint_value) == len(joint_names), "target_joint_value must have length equal to joint count"
            wp1 = JointWaypoint(joint_names, initial_joint_value)
            wp2 = JointWaypoint(joint_names, target_joint_value)

            manip_info = ManipulatorInfo()
            manip_info.manipulator = robot_name

            start_instruction = PlanInstruction(Waypoint(wp1), PlanInstructionType_START, "DEFAULT")
            plan_f1 = PlanInstruction(Waypoint(wp2), PlanInstructionType_FREESPACE, "DEFAULT")

            program = CompositeInstruction("DEFAULT")
            program.setStartInstruction(Instruction(start_instruction))
            program.setManipulatorInfo(manip_info)
            program.append(Instruction(plan_f1))

            request = ProcessPlanningRequest()
            request.name = FREESPACE_PLANNER_NAME
            request.instructions = Instruction(program)
            
            response = self._planning_server.run(request)
            response.wait()           
            
            assert response.interface.isSuccessful()

            tesseract_trajectory = flatten(response.getResults().as_CompositeInstruction())

            #fwd_kin = self._t_env.getManipulatorManager().getFwdKinematicSolver(robot_name)
            #limits = fwd_kin.getLimits()

            #self._spline_param.compute(tesseract_trajectory, limits.velocity_limits, 
            #    limits.acceleration_limits, speed_perc/100.0, speed_perc/100.0)

            t_scale = 1.0
            if speed_perc < 100:
                t_scale = 100.0/speed_perc

            trajectory_wps = []
            for i in range(len(tesseract_trajectory)):
                instr = tesseract_trajectory[i]
                assert isMoveInstruction(instr)
                wp = instr.as_MoveInstruction().getWaypoint()
                assert isStateWaypoint(wp)
                state_wp = wp.as_StateWaypoint()
                trajectory_wp = self._trajectory_wp_type()
                trajectory_wp.joint_position = state_wp.position.flatten()
                trajectory_wp.time_from_start = state_wp.time * t_scale
                trajectory_wps.append(trajectory_wp)

            trajectory = self._trajectory_type()
            trajectory.joint_names = joint_names
            trajectory.waypoints = trajectory_wps

            self._update_viewer(robot_name,tesseract_trajectory)

            return PlanMoveResultGenerator(trajectory, self._node)

    def _update_viewer(self, robot_name = None, trajectory = None):
        if robot_name is None:
            joint_names = list(self._t_env.getActiveJointNames())
            joint_values = self._t_env.getCurrentJointValues().flatten()

            self._viewer.update_joint_positions(joint_names, joint_values)
        else:
            self._viewer.update_trajectory(trajectory)


class PlanMoveResultGenerator:
    def __init__(self, trajectory, node):
        self._trajectory = trajectory
        self._node = node
        self._closed = False
        self._aborted = False
        self._lock = threading.Lock()

        self._action_consts = self._node.GetConstants("com.robotraconteur.action")
        self._action_success = self._action_consts["ActionStatusCode"]["complete"]
        self._plan_move_result_type = self._node.GetStructureType("tech.pyri.robotics.tesseract_planner.PlanMoveResult")

    def Next(self):
        with self._lock:
            if self._aborted:
                raise RR.OperationAbortedException("Generator has been aborted")            
            if self._closed:
                raise RR.StopIterationException("")

            self._closed = True

            res = self._plan_move_result_type()
            res.action_status = self._action_success
            res.trajectory = self._trajectory

            return res

    def Close(self):
        with self._lock:
            self._closed = True

    def Abort(self):
        with self._lock:
            self._aborted = True


class GazeboModelResourceLocatorFn:
	def __init__(self):
		model_env_path = os.environ["GAZEBO_MODEL_PATH"]
		self.model_paths = model_env_path.split(os.pathsep)
		assert len(self.model_paths) != 0, "No GAZEBO_MODEL_PATH specified!"
		for p in self.model_paths:
			assert os.path.isdir(p), "GAZEBO_MODEL_PATH directory does not exist: %s" % p

	def __call__(self,url):
		try:
			url_match = re.match(r"^model:\/\/(\w+)\/(.+)$",url)
			if (url_match is None):
				assert False, "Invalid Gazebo model resource url %s" % url
			model_name = url_match.group(1)
			resource_path = os.path.normpath(url_match.group(2))

			for p in self.model_paths:

				fname = os.path.join(p, model_name, resource_path )
				if not os.path.isfile(fname):
					continue
				return fname

			assert False, "Could not find requested resource %s" % url
		except:
			traceback.print_exc()
			return ""

def GazeboModelResourceLocator():
	locator_fn = SimpleResourceLocatorFn(GazeboModelResourceLocatorFn())
	locator = SimpleResourceLocator(locator_fn)
	locator_fn.__disown__()
	return locator

def main():

    parser = argparse.ArgumentParser(description="PyRI Robotics Tesseract Motion Planner Service")
    parser.add_argument("--urdf-file", type=argparse.FileType('r'),default=None,required=True,help="URDF file for scene (required)")
    parser.add_argument("--srdf-file", type=argparse.FileType('r'),default=None,required=True,help="SRDF file for scene (required)")
    parser.add_argument("--viewer-http-port",type=int,default=8001,help="HTTP TCP/IP Listen Port for Tesseract viewer(default 8001)")
    # Add version here so it doesn't raise an error...
    version1 = importlib.metadata.version('pyri-tesseract-planner')
    parser.add_argument('-V', '--version', action='version', version=f'%(prog)s {version1}')
    args, _ = parser.parse_known_args()

    with args.urdf_file:
        urdf_file_text = args.urdf_file.read()

    with args.srdf_file:
        srdf_file_text = args.srdf_file.read()

    t_env = Environment()

    locator = GazeboModelResourceLocator()
    t_env.init(urdf_file_text, srdf_file_text, locator)

    viewer = TesseractViewer(server_address=('',args.viewer_http_port))

    viewer.update_environment(t_env, [0,0,0])
    viewer.start_serve_background()

    with PyriServiceNodeSetup("tech.pyri.robotics.tesseract_planner", 55923, \
        extra_service_defs=[(__package__,'tech.pyri.robotics.tesseract_planner.robdef')], \
        default_info = (__package__,"pyri_tesseract_planner_service_default_info.yml"), \
        display_description="PyRI Robotics Tesseract Motion Planner Service", no_device_manager=True, \
        ) as service_node_setup:
        
        planner_inst = TesseractPlannerService_impl(device_info=service_node_setup.device_info_struct, t_env=t_env, viewer=viewer, node = RRN)
        
        service_node_setup.register_service("tesseract_planner","tech.pyri.robotics.tesseract_planner.TesseractPlannerService",planner_inst)

        #Wait for the user to shutdown the service
        service_node_setup.wait_exit()
        
if __name__ == '__main__':
    main()
