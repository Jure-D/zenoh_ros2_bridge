# export PYTHONPATH=$PYTHONPATH:/home/jure/ros2_ws/colcon_env/lib/python3.12/site-packages

import zenoh
import msgpack

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped, Twist, Pose
from mocap4r2_msgs.msg import RigidBodies, RigidBody

class ZenohROS2Bridge(Node):
    def __init__(self):
        super().__init__('lampa_zenoh_bridge')

        self.declare_parameter('active_robots', rclpy.Parameter.Type.STRING_ARRAY)
        self.active_robots = self.get_parameter('active_robots').get_parameter_value().string_array_value
        # self.get_logger().info(f"PARAMETERS: {}")

        # self.declare_parameter('robot_number', 3)
        # self.robot_namespace = f"/turtle{self.get_parameter('robot_number').value}"
        # self.get_logger().info(f"{self.robot_namespace[1:]}")

        self.cmd_vel_publishers = {}
        for robot in self.active_robots:
            self.cmd_vel_publishers[robot] = self.create_publisher(TwistStamped, f'/{robot}/cmd_vel', 10)

        # self.cmd_vel_publisher = self.create_publisher(TwistStamped, f'{self.robot_namespace}/cmd_vel', 10)
        self.cmd_vel_msg = TwistStamped()
        self.cmd_vel_msg.header.frame_id = ""

        self.zenoh_session = zenoh.open(zenoh.Config())
        self.zenoh_cmd_vel_subscription = self.zenoh_session.declare_subscriber(f"*/cmd_vel", self.zenoh_cmd_vel_handler)

        self.rigid_bodies_subscription = self.create_subscription(
            RigidBodies,
            '/rigid_bodies',
            self.rigid_bodies_handler,
            10)
        
        self.optitrack_publishers = {}
        for robot in self.active_robots:
            self.optitrack_publishers[robot] = self.zenoh_session.declare_publisher(f"{robot}/pose")

    def rigid_bodies_handler(self, msg):
        data = {
            "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec*10e-9,
        }

        for rigid_body in msg.rigidbodies:
            robot = f"turtle{int(rigid_body.rigid_body_name)}"
            self.get_logger().info(f"Updating \"{robot}\"")
            data["pose"] = {
                "position": {
                    "x": rigid_body.pose.position.x,
                    "y": rigid_body.pose.position.y,
                    "z": rigid_body.pose.position.z
                },
                "orientation": {
                    "x": rigid_body.pose.orientation.x,
                    "y": rigid_body.pose.orientation.y,
                    "z": rigid_body.pose.orientation.z,
                    "w": rigid_body.pose.orientation.w
                },
            }

            msg_packed = msgpack.packb(data, use_bin_type=True)
            self.optitrack_publishers[robot].put(msg_packed)


    def zenoh_cmd_vel_handler(self, sample):
        zenoh_data = msgpack.unpackb(sample.payload.to_bytes(), raw=False)
        # self.get_logger().info(f"{zenoh_data}")
        self.cmd_vel_msg.header.stamp = rclpy.time.Time().to_msg()
        self.cmd_vel_msg.twist.linear.x = zenoh_data["linear"]["x"]
        self.cmd_vel_msg.twist.linear.y = zenoh_data["linear"]["y"]
        self.cmd_vel_msg.twist.linear.z = zenoh_data["linear"]["z"]
        self.cmd_vel_msg.twist.angular.x = zenoh_data["angular"]["x"]
        self.cmd_vel_msg.twist.angular.y = zenoh_data["angular"]["y"]
        self.cmd_vel_msg.twist.angular.z = zenoh_data["angular"]["z"]
        self.get_logger().info(f"{str(sample.key_expr).split("/")[0]}")
        self.cmd_vel_publishers[str(sample.key_expr).split("/")[0]].publish(self.cmd_vel_msg)


def main(args=None):
    rclpy.init(args=args)

    lampa_zenoh_bridge = ZenohROS2Bridge()

    rclpy.spin(lampa_zenoh_bridge)

    lampa_zenoh_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()