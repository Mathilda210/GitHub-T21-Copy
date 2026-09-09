import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from px_mc_msgs.msg import ChassisStatus


K_NODE_NAME = "waist_depth_camera_test_node"
K_CHASSIS_STATUS_TOPIC = "/chassis_status"


class GetChassisStatusNode(Node):
    def __init__(self):
        super().__init__(K_NODE_NAME)

        self.subscription = self.create_subscription(
            ChassisStatus,
            K_CHASSIS_STATUS_TOPIC,
            self.chassis_status_callback,
            qos_profile_system_default
        )

    def chassis_status_callback(self, msg):
        self.get_logger().info(
            f"Chassis moving at velocity=[{msg.linear:.6f}] "
            f"angular=[{msg.angular:.6f}] "
            f"direction=[{msg.direction:.6f}]"
        )

        self.get_logger().info(
            f"Chassis imu reports linear_acceleration "
            f"x=[{msg.imu.linear_acceleration.x:.6f}] "
            f"y=[{msg.imu.linear_acceleration.y:.6f}] "
            f"z=[{msg.imu.linear_acceleration.z:.6f}]"
        )

        self.get_logger().info(
            f"Chassis imu reports angular_velocity "
            f"x=[{msg.imu.angular_velocity.x:.6f}] "
            f"y=[{msg.imu.angular_velocity.y:.6f}] "
            f"z=[{msg.imu.angular_velocity.z:.6f}]"
        )

        self.get_logger().info(
            f"Chassis imu reports quaternion "
            f"x=[{msg.imu.orientation.x:.6f}] "
            f"y=[{msg.imu.orientation.y:.6f}] "
            f"z=[{msg.imu.orientation.z:.6f}] "
            f"w=[{msg.imu.orientation.w:.6f}]"
        )

        self.get_logger().info(
            f"Chassis odom reports cordinates "
            f"x=[{msg.odom.pose.pose.orientation.x:.6f}] "
            f"y=[{msg.odom.pose.pose.orientation.y:.6f}] "
            f"z=[{msg.odom.pose.pose.orientation.z:.6f}]"
        )

        self.get_logger().info(
            f"Chassis odom reports quaternion "
            f"x=[{msg.odom.pose.pose.orientation.x:.6f}] "
            f"y=[{msg.odom.pose.pose.orientation.y:.6f}] "
            f"z=[{msg.odom.pose.pose.orientation.z:.6f}] "
            f"w=[{msg.odom.pose.pose.orientation.w:.6f}]"
        )

        self.get_logger().info(
            f"Chassis odom reports linear velocity "
            f"x=[{msg.odom.twist.twist.linear.x:.6f}] "
            f"y=[{msg.odom.twist.twist.linear.y:.6f}], "
            f"angular velocity z=[{msg.odom.twist.twist.angular.z:.6f}]"
        )


def main(args=None):
    rclpy.init(args=args)

    node = GetChassisStatusNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()