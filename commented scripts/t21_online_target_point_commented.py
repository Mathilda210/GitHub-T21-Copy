



# ---------------------------- IMPORTS ---------------------------
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
# Custom intefaces
from px_mc_msgs.msg import (OnlineMotionCommand, TrajectoryPoint, ArmConstraintParameters)






# --------------------------- ONLINE TRAJECTORY PUBLISHER ---------------------------
class OnlineTrajectoryPublisher(Node):
    '''ROS 2 node that publishes an online motion command to move the robot's arms to specified target positions.'''
    def __init__(self):
        super().__init__('online_trajectory_publisher')

        self.publisher = self.create_publisher(
            # Message type
            OnlineMotionCommand,
            # Topic name
            '/external/online_motion_command',
            # QoS history depth
            10
        )

        self.arm_indices = [0, 1, 2]

        # Define home target positions for each arm (in radians for joints)
        # self.target_positions = {
        #     0: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #     1: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #     2: [0.0, 0.0, 0.0, 0.0],
        #     3: [0.0, 0.0],
        #     4: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #     5: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # }

        # Define target positions for each arm (in radians for joints)
        self.target_positions = {
            # 7 motors in each arm
            0: [0.0, 0.6, 0.0, -1.57, 0.0, -0.3, 0.0],
            1: [0.0, -0.6, 0.0, -1.57, 0.0, 0.3, 0.0],
            # 6 motors to pilot the waist 
            2: [0.0, 0.0, 0.2, -0.2, 0.0, 0.0],
            # 2 motors to pilot the head arm
            3: [0.3, 0.3],
            # 7 motors to pilot each hand 
            4: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            5: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        }

        # Publish command once after a short delay to ensure system is ready
        self.publish_timer = self.create_timer(1.0, self.publish_command)
        self.command_sent = False


    def publish_command(self):
        '''Publish an online motion command to move the robot's arms to the specified target positions.'''
        if self.command_sent:
            return

        msg = OnlineMotionCommand()
        msg.type = OnlineMotionCommand.JOINT_TRAJECTORY
        msg.target_arm_indices = self.arm_indices

        # Create target points 
        target_points = []

        arm_names = {
            0: "Left Arm",
            1: "Right Arm",
            2: "Waist Arm",
            3: "Head Arm",
            4: "Left Hand",
            5: "Right Hand"
        }

        # As defined previously the arm indices are: 0,1 and 2 in this case
        for arm_idx in self.arm_indices:
            point = TrajectoryPoint()
            point.positions = self.target_positions[arm_idx]
            point.velocities = [0.0] * len(point.positions)

            target_points.append(point)

            self.get_logger().info(
                f"{arm_names[arm_idx]} target positions: "
                f"{[f'{p:.3f}' for p in point.positions]}"
            )

        msg.target_point = target_points
        msg.planner = OnlineMotionCommand.PLANNER_RUCKIG

        # Set constraint parameters per arm
        # Format: ArmConstraintParameters array,
        # one per target_arm_indices
        msg.max_velocity = []
        msg.max_acceleration = []
        msg.max_jerk = []

        for arm_idx in self.arm_indices:
            if arm_idx in [0, 1]:
                # Left or right arm (7 joints)
                # Single value for all joints
                msg.max_velocity.append(
                    ArmConstraintParameters(values=[0.4])
                )
                msg.max_acceleration.append(
                    ArmConstraintParameters(values=[0.5])
                )
                msg.max_jerk.append(
                    ArmConstraintParameters(values=[5.0])
                )

            elif arm_idx == 2:
                # Waist arm (6 joints)
                # Per-joint values
                msg.max_velocity.append(
                    ArmConstraintParameters(
                        values=[0.1, 0.1, 0.05, 0.15, 0.13, 0.14]
                    )
                )
                msg.max_acceleration.append(
                    ArmConstraintParameters(
                        values=[0.4, 0.5, 0.1, 0.4, 0.5, 0.3]
                    )
                )
                msg.max_jerk.append(
                    ArmConstraintParameters(
                        values=[4.0, 5.0, 3.0, 4.0, 5.0, 3.0]
                    )
                )

            else:
                # Other arms(head and hands) - use default values
                msg.max_velocity.append(
                    ArmConstraintParameters(values=[0.02])
                )
                msg.max_acceleration.append(
                    ArmConstraintParameters(values=[0.5])
                )
                msg.max_jerk.append(
                    ArmConstraintParameters(values=[5.0])
                )

        # Alternative: Use default values by leaving arrays empty
        # msg.max_velocity = []
        # msg.max_acceleration = []
        # msg.max_jerk = []

        msg.is_whole_body = False
        msg.reference_frame = "world_frame"
        msg.command_id = 1
        msg.need_interpolation = True
        msg.is_urgent = False
        msg.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(msg)

        self.command_sent = True

        self.get_logger().info(
            f"Published online motion command for arms "
            f"{self.arm_indices}"
        )



# --------------------------- MAIN FUNCTION ---------------------------
def main(args=None):
    rclpy.init(args=args)
    node = OnlineTrajectoryPublisher()

    # We use a MultiThreadedExecutor to allow the node to handle multiple callbacks concurrently.
    # In this specific script there is only one callback so rlcpy.spin(node) would work as well
    # but using a MultiThreadedExecutor is a good practice for more complex nodes.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()                            