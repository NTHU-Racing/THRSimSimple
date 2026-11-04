"""Small teaching example: fixed lookahead + proportional heading control.

No simulator imports, numpy, YAML, speed controller or trajectory optimizer.
The car cruises at a low constant speed and requests zero at the finish node.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty
from thr_sim_msgs.msg import RefPath, VehCtrl, VehState


def control_command(points, finish_index, pose, index, speed=3.0,
                    lookahead=4.0, gain=1.0):
    """Return (velocity, steer, progress_index); all angles are radians."""
    if not points or pose is None:
        return 0.0, 0.0, 0
    x, y, yaw = pose

    # Find the closest nearby node. Searching locally avoids jumping straight
    # to the finish where a circle overlaps its starting point.
    end = min(len(points), index + 20)
    nearest = min(range(index, end),
                  key=lambda i: (points[i][0] - x)**2 + (points[i][1] - y)**2)
    if nearest >= finish_index:
        return 0.0, 0.0, nearest

    # Walk a fixed distance forward along the nodes to choose a steering target.
    target, distance = nearest, 0.0
    while target + 1 < len(points) and distance < lookahead:
        a, b = points[target], points[target + 1]
        distance += math.hypot(b[0] - a[0], b[1] - a[1])
        target += 1
    tx, ty = points[target]
    error = math.atan2(ty - y, tx - x) - yaw
    error = math.atan2(math.sin(error), math.cos(error))  # wrap to [-pi, pi]
    steer = max(-0.6, min(0.6, gain * error))
    return speed, steer, nearest


class DemoDriver(Node):
    def __init__(self):
        super().__init__('demo_driver')
        self.speed = self.declare_parameter('speed', 15.0).value       # m/s
        self.lookahead = self.declare_parameter('lookahead', 4.0).value  # m
        self.gain = self.declare_parameter('gain', 1.0).value
        self.points, self.finish_index, self.index, self.pose = [], 0, 0, None
        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(RefPath, '/path', self.on_path, path_qos)
        self.create_subscription(VehState, '/veh_state', self.on_state, 10)
        self.create_subscription(Empty, '/reset', self.on_reset, 10)
        self.publisher = self.create_publisher(VehCtrl, '/veh_ctrl', 10)
        self.create_timer(0.05, self.control)  # 20 Hz, including while idle
        self.get_logger().info('Demo ready. Start run_sim.bash or evaluate.bash separately.')

    def on_path(self, msg):
        self.points = [(p.x, p.y) for p in msg.nodes]
        self.finish_index, self.index = msg.finish_index, 0
        if len(self.points) < 2 or self.finish_index >= len(self.points):
            self.points = []  # empty path = no task: request zero speed

    def on_state(self, msg):
        self.pose = (msg.x, msg.y, msg.yaw)

    def on_reset(self, _msg):
        self.index, self.pose = 0, None
        # Keep the path: /path and /reset can arrive in either order.

    def control(self):
        velocity, steer, self.index = control_command(
            self.points, self.finish_index, self.pose, self.index,
            self.speed, self.lookahead, self.gain)
        msg = VehCtrl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.velocity, msg.steer = float(velocity), float(steer)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DemoDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
