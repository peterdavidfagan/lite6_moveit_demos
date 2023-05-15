import rclpy
from multiprocessing import Process

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

from dataclasses import dataclass
from moveit.servo_client.teleop import TeleopDevice


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
        except Exception as e:
            self.logger.info.error(e)
            print(e)

    def record():
        pass


if __name__=="__main__":
    rclpy.init()

    # Instantiate and activate the teleop device
    ps4 = PS4DualShockTeleop('link_eef')
    ps4.start_teleop()
    rclpy.spin(minimal_publisher)
    
    # stop teleoperation and shutdown
    ps4.stop_teleop()
    minimal_publisher.destroy_node()
    rclpy.shutdown()

