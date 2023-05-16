#!/usr/bin/env python3
"""
Start PS4 teleop.
"""
from multiprocessing import Process
from dataclasses import dataclass
import math

import rclpy
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

from moveit_configs_utils import MoveItConfigsBuilder
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.core.robot_state import RobotState
from moveit.core.robot_state import RobotState
from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.servo_client.teleop import TeleopDevice
from moveit.servo_client.devices.ps4_dualshock import PS4DualShock

from xarm.wrapper import XArmAPI

class PS4DualShockTeleop(TeleopDevice):
    "A class which encapsulates teleoperation functionalities for ps4 dualshock device."

    def __init__(
        self,
        ee_frame_name: str,
        node_name: str = "ps4_dualshock_teleop",
        device_name: str = "ps4_dualshock",
        device_config: PS4DualShock = PS4DualShock(),
    ):
        super().__init__(
            node_name=node_name,
            device_name=device_name,
            device_config=device_config,
            ee_frame_name=ee_frame_name,
        )
        self.logger = rclpy.logging.get_logger("ps4_dualshock_teleop")
        
        # hardcode the ip address for now
        self.xarm = XArmAPI("192.168.1.156")
        
        # status flags
        self.gripper_open = True

        # connect to xarm and enable motion
        self.xarm.connect()
        self.xarm.motion_enable(enable=True)


    def publish_command(self, data):
        """
        Publishes the teleop command.
        """
        try:
            # convert joy data to twist command
            twist = TwistStamped()
            twist.twist.linear.z = data.axes[self.device_config.Axes.RIGHT_STICK_Y]
            twist.twist.linear.y = data.axes[self.device_config.Axes.RIGHT_STICK_X]

            lin_x_right = -0.5 * (data.axes[self.device_config.Axes.RIGHT_TRIGGER])
            lin_x_left = 0.5 * (data.axes[self.device_config.Axes.LEFT_TRIGGER])
            twist.twist.linear.x = lin_x_right + lin_x_left

            twist.twist.angular.y = data.axes[self.device_config.Axes.LEFT_STICK_Y]
            twist.twist.angular.x = data.axes[self.device_config.Axes.LEFT_STICK_X]

            roll_positive = data.buttons[self.device_config.Buttons.R1]
            roll_negative = -1 * data.buttons[self.device_config.Buttons.L1]
            twist.twist.angular.z = float(roll_positive + roll_negative)

            twist.header.frame_id = self.ee_frame_name
            twist.header.stamp = self.get_clock().now().to_msg()
            self.twist_publisher.publish(twist)
            
            # open/close gripper
            if data.buttons[self.device_config.Buttons.O] == 1:
                if self.gripper_open:
                    self.xarm.close_lite6_gripper()
                    self.gripper_open = False
                else:
                    self.xarm.open_lite6_gripper()
                    self.gripper_open = True

            # stop gripper noise
            if data.buttons[self.device_config.Buttons.TRIANGLE] == 1:
                self.xarm.stop_lite6_gripper()

        except Exception as e:
            self.logger.info.error(e)
            print(e)

    def record():
        pass


if __name__=="__main__":
    rclpy.init()
    
    # start moveit_py
    moveit_config = (
        MoveItConfigsBuilder(robot_name="lite6", package_name="moveit_resources_lite6_moveit_config")
        .robot_description_semantic(file_path="config/lite6.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description(file_path=get_package_share_directory("moveit_resources_lite6_description")
                          + "/urdf/lite6.urdf")
        .moveit_cpp(
            file_path=get_package_share_directory("lite6_moveit_demos")
            + "/config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
    ).to_dict()

    lite6 = MoveItPy(node_name="moveit_py", config_dict=moveit_config)
    lite6_arm = lite6.get_planning_component("lite6")

    # Instantiate and activate the teleop device
    ps4 = PS4DualShockTeleop(ee_frame_name='link_eef', moveit=lite6)
    ps4.start_teleop()

    # TODO: incorporate shutdown teleop into button press
    import time
    time.sleep(100000.0)

    # stop teleoperation and shutdown
    ps4.stop_teleop()
    ps4.destroy_node()
    rclpy.shutdown()

