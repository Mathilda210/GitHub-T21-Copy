import rclpy
import threading
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from px_mc_msgs.msg import ChassisControl
import sys

K_NODE_NAME = "control_chassis_movement_node"
K_CHASSIS_CONTROL_TOPIC = "/chassis_control_joy"

def move_forward(node, publisher):
    duration_ns = 2 * 1e9
    start_time = node.get_clock().now()
    rate = node.create_rate(100)

    while rclpy.ok() and (node.get_clock().now() - start_time).nanoseconds < duration_ns:
        msg = ChassisControl()
        msg.linear_x = 0.2
        publisher.publish(msg)
        rate.sleep()

def move_backward(node, publisher):
    duration_ns = 2 * 1e9
    start_time = node.get_clock().now()
    rate = node.create_rate(100)

    while rclpy.ok() and (node.get_clock().now() - start_time).nanoseconds < duration_ns:
        msg = ChassisControl()
        msg.linear_x = -0.2
        publisher.publish(msg)
        rate.sleep()

def rotate_left(node, publisher):
    duration_ns = 4 * 1e9
    start_time = node.get_clock().now()
    rate = node.create_rate(100)

    while rclpy.ok() and (node.get_clock().now() - start_time).nanoseconds < duration_ns:
        msg = ChassisControl()
        msg.angular = 0.4
        publisher.publish(msg)
        rate.sleep()

def rotate_right(node, publisher):
    duration_ns = 4 * 1e9
    start_time = node.get_clock().now()
    rate = node.create_rate(100)

    while rclpy.ok() and (node.get_clock().now() - start_time).nanoseconds < duration_ns:
        msg = ChassisControl()
        msg.angular = -0.4
        publisher.publish(msg)
        rate.sleep()

def emergency_stop(publisher):
    msg = ChassisControl()
    msg.e_stop = ChassisControl.EMERGENCY_STOP_TRIGGERED
    publisher.publish(msg)

def release_emergency_stop(publisher):
    msg = ChassisControl()
    msg.e_stop = ChassisControl.EMERGENCY_STOP_RELEASE
    publisher.publish(msg)

def demo(node, publisher):
    print("Please select robot movements:")
    print("1: forward")
    print("2: backward")
    print("3: rotate left")
    print("4: rotate right")
    print("5: emergency stop")
    print("6: release emergency stop")
    print("q: quit")

    while rclpy.ok():
        choice = input().strip()

        if choice == 'q':
            break
        elif choice == '1':
            move_forward(node, publisher)
        elif choice == '2':
            move_backward(node, publisher)
        elif choice == '3':
            rotate_left(node, publisher)
        elif choice == '4':
            rotate_right(node, publisher)
        elif choice == '5':
            emergency_stop(publisher)
        elif choice == '6':
            release_emergency_stop(publisher)
        else:
            print("Invalid")

def main(args=None):
    rclpy.init(args=args)

    node = Node(K_NODE_NAME)

    chassis_control_publisher = node.create_publisher(
        ChassisControl,
        K_CHASSIS_CONTROL_TOPIC,
        qos_profile_system_default
    )

    thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True
    )
    thread.start()

    try:
        demo(node, chassis_control_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        thread.join()

if __name__ == '__main__':
    main()                         