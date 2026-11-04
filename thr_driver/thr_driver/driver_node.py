"""Demo driverless node using simple P control.

Subscribes:
    /path      (thr_sim_msgs/RefPath, transient-local) nodes + finish index;
               an empty path means "no task": hold the vehicle still
    /veh_state (thr_sim_msgs/VehState)
    /reset     (std_msgs/Empty)
Publishes:
    /veh_ctrl  (thr_sim_msgs/VehCtrl)

Steering: P control on the heading error toward the path point a constant
`lookahead` distance ahead of the vehicle's position on the path
    steer = kp_steer * wrap(atan2(target - position) - yaw)
Speed: constant `velocity`, 0 once the vehicle has passed the finish node.

Parameters have defaults below (no YAML); override with e.g.
    ros2 run thr_driver driver --ros-args -p velocity:=5.0
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Empty

from thr_sim_msgs.msg import RefPath, VehCtrl, VehState
from thr_driver.path_tracker import PathTracker, wrap_angle


class DriverNode(Node):
    def __init__(self):
        super().__init__('driver')
        p = self.declare_parameter
        self.kp_steer = p('kp_steer', 1.5).value      # steer [rad] per rad of heading error
        self.velocity = p('velocity', 10.0).value     # constant speed [m/s]
        self.lookahead = p('lookahead', 4.5).value    # distance ahead along the path [m]
        self.max_steer = p('max_steer', 0.6).value    # [rad]
        rate = p('control_rate', 20.0).value          # [Hz]

        self.proj = None
        self.veh = None

        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(RefPath, '/path', self.on_path, latched)
        self.create_subscription(VehState, '/veh_state', self.on_state, 10)
        self.create_subscription(Empty, '/reset', self.on_reset, 10)
        self.ctrl_pub = self.create_publisher(VehCtrl, '/veh_ctrl', 10)
        self.create_timer(1.0 / rate, self.on_control)
        self.get_logger().info('Driver ready, waiting for /path and /veh_state')

    def on_path(self, msg: RefPath):
        if len(msg.nodes) < 2:
            self.proj = None
            self.get_logger().info('No task (empty path): holding still')
            return
        pts = np.array([[pt.x, pt.y] for pt in msg.nodes])
        self.proj = PathTracker(pts)
        self.s_finish = float(self.proj.s[min(msg.finish_index, len(pts) - 1)])
        self.get_logger().info(
            f'Received path: {len(pts)} nodes, finish at {self.s_finish:.1f} m')

    def on_state(self, msg: VehState):
        self.veh = msg

    def on_reset(self, _msg):
        self.veh = None
        if self.proj is not None:
            self.proj.reset()
        self.get_logger().info('Reset received')

    def on_control(self):
        if self.proj is None:
            self.publish(0.0, 0.0)   # no task: stand still
            return
        if self.veh is None:
            return
        x, y, yaw = self.veh.x, self.veh.y, self.veh.yaw
        s, _, _ = self.proj.project(x, y)
        v = self.velocity if s < self.s_finish else 0.0   # 0 once past the finish

        tx, ty = self.proj.point_at(s + self.lookahead)
        heading_err = wrap_angle(math.atan2(ty - y, tx - x) - yaw)
        steer = max(-self.max_steer, min(self.max_steer,
                                         self.kp_steer * heading_err))

        self.publish(v, steer)

    def publish(self, v, steer):
        msg = VehCtrl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.velocity = float(v)
        msg.steer = float(steer)
        self.ctrl_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
