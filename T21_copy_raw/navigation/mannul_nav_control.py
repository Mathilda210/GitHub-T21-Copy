import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from px_mc_msgs.msg import ChassisControl

# Fréquence de contrôle recommandée : 50 Hz (période 20 ms),
# conforme à l'exigence > 20 Hz
CONTROL_FREQUENCY = 50.0  # Hz
CONTROL_PERIOD = 1.0 / CONTROL_FREQUENCY  # s

# Limites de vitesse de sécurité
MAX_LINEAR_X = 0.5   # m/s
MAX_LINEAR_Y = 0.3   # m/s (séries P uniquement)
MAX_ANGULAR = 0.8    # rad/s


class ChassisMovementController(Node):
    def __init__(self, linear_x, linear_y, angular, duration, e_stop):
        super().__init__('t21_control_chassis_movement')

        self.linear_x = self._clamp(
            linear_x,
            -MAX_LINEAR_X,
            MAX_LINEAR_X
        )
        self.linear_y = self._clamp(
            linear_y,
            -MAX_LINEAR_Y,
            MAX_LINEAR_Y
        )
        self.angular = self._clamp(
            angular,
            -MAX_ANGULAR,
            MAX_ANGULAR
        )

        self.duration = max(0.0, duration)
        self.e_stop = e_stop

        self._elapsed = 0.0
        self._stopped = False

        self._pub = self.create_publisher(
            ChassisControl,
            '/chassis_control',
            10
        )

        self._timer = self.create_timer(
            CONTROL_PERIOD,
            self._timer_callback
        )

        self.get_logger().info(
            f'底盘手动控制启动: '
            f'linear_x={self.linear_x:.2f}, '
            f'linear_y={self.linear_y:.2f}, '
            f'angular={self.angular:.2f}, '
            f'duration={self.duration:.1f}s, '
            f'e_stop={self.e_stop}'
        )

    def _clamp(self, value, min_val, max_val):
        return max(min_val, min(max_val, value))

    def _build_command(self):
        msg = ChassisControl()

        # Gestion de l'arrêt d'urgence logiciel
        if self.e_stop:
            msg.software_e_stop = (
                ChassisControl.EMERGENCY_STOP_TRIGGERED
            )
            msg.linear_x = 0.0
            msg.linear_y = 0.0
            msg.angular = 0.0
        else:
            msg.software_e_stop = (
                ChassisControl.EMERGENCY_STOP_RELEASE
            )
            msg.linear_x = self.linear_x
            msg.linear_y = self.linear_y
            msg.angular = self.angular

        # Arrêt d'urgence matériel : laissé inchangé
        msg.e_stop = ChassisControl.EMERGENCY_STOP_NULL

        return msg

    def _timer_callback(self):
        if self._stopped:
            return

        # Arrêt automatique à la fin de la durée demandée
        if self.duration > 0.0 and self._elapsed >= self.duration:
            self._stop_chassis()
            return

        msg = self._build_command()
        self._pub.publish(msg)

        self._elapsed += CONTROL_PERIOD

    def _stop_chassis(self):
        if self._stopped:
            return

        stop_msg = ChassisControl()
        stop_msg.e_stop = ChassisControl.EMERGENCY_STOP_NULL
        stop_msg.software_e_stop = (
            ChassisControl.EMERGENCY_STOP_RELEASE
        )
        stop_msg.linear_x = 0.0
        stop_msg.linear_y = 0.0
        stop_msg.angular = 0.0

        # Publier plusieurs fois la commande d'arrêt
        # pour garantir sa réception
        for _ in range(5):
            self._pub.publish(stop_msg)
            self.get_clock().sleep_for(
                rclpy.duration.Duration(
                    seconds=CONTROL_PERIOD
                )
            )

        self._stopped = True
        self.get_logger().info('底盘已停止')

    def destroy_node(self):
        self._stop_chassis()
        super().destroy_node()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='TORA Double One 底盘手动控制示例',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    parser.add_argument(
        '--linear_x',
        type=float,
        default=0.0,
        help='Vitesse linéaire X en m/s '
             '(avant positif, arrière négatif)'
    )

    parser.add_argument(
        '--linear_y',
        type=float,
        default=0.0,
        help='Vitesse linéaire Y en m/s '
             '(uniquement pour les châssis omnidirectionnels série P)'
    )

    parser.add_argument(
        '--angular',
        type=float,
        default=0.0,
        help='Vitesse angulaire en rad/s '
             '(positive vers la gauche, négative vers la droite)'
    )

    parser.add_argument(
        '--duration',
        type=float,
        default=0.0,
        help='Durée du mouvement en secondes ; '
             '0 signifie exécution jusqu’à Ctrl+C'
    )

    parser.add_argument(
        '--e_stop',
        action='store_true',
        help='Déclenche l’arrêt d’urgence logiciel '
             '(les vitesses sont alors ignorées)'
    )

    return parser.parse_args(argv)


def main(args=None):
    parsed = parse_args(args[1:] if args else None)

    rclpy.init(args=args)

    node = ChassisMovementController(
        linear_x=parsed.linear_x,
        linear_y=parsed.linear_y,
        angular=parsed.angular,
        duration=parsed.duration,
        e_stop=parsed.e_stop
    )

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            '收到 Ctrl+C，正在停止底盘...'
        )

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)