import rclpy
from rclpy.node import Node
from px_perception_msgs.srv import ChassisState


class ChassisControlClient(Node):
    def __init__(self):
        super().__init__('chassis_control_client')

        self.cli = self.create_client(
            ChassisState,
            '/chassis/chassis_state'
        )

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                '等待 /chassis/chassis_state 服务...'
            )

    def call(
        self,
        system_state=0,
        emergency_stop=0,
        software_e_stop=0
    ):
        req = ChassisState.Request()

        req.system_state = system_state
        req.emergency_stop = emergency_stop
        req.software_e_stop = software_e_stop

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        return future.result()


if __name__ == '__main__':
    rclpy.init()

    client = ChassisControlClient()

    # Passer en mode automatique
    resp = client.call(
        system_state=ChassisState.Request.STATE_AUTO
    )

    print(f'结果: {resp.result}')

    client.destroy_node()
    rclpy.shutdown()