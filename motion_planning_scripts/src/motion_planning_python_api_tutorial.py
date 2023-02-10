#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
import sys
import threading
import time

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Constraints

from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
    MultiPipelinePlanRequestParameters,
)
from moveit.core.robot_state import RobotState
from moveit.core.kinematic_constraints import construct_joint_constraint

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    moveit = MoveItPy(node_name="moveit_py")
    lite6 = moveit.get_planning_component("lite6")
    logger.info("MoveItPy instance created")

    ###########################################################################
    # Plan 1 - set states with predefined string
    ###########################################################################
    lite6.set_start_state_to_current_state()

    # set constraints message
    joint_values = {
        "joint1": math.radians(100),
        "joint2": math.radians(10.4),
        "joint3": math.radians(31.1),
        "joint4": math.radians(-1.5),
        "joint5": math.radians(21.5),
        "joint6": math.radians(1.3),
    }
    robot_state = RobotState(moveit.get_robot_model())
    robot_state.joint_positions = joint_values
    joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=moveit.get_robot_model().get_joint_model_group("lite6"),
    )
    lite6.set_goal_state(motion_plan_constraints=[joint_constraint])

    # plan to goal
    plan_result = lite6.plan()

    # execute the plan
    if plan_result:
        lite6.execute()

if __name__ == "__main__":
    main()
