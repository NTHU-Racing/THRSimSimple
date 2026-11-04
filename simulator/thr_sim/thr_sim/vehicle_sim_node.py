"""Vehicle simulator node.

Subscribes:
    /veh_ctrl  (thr_sim_msgs/VehCtrl)  velocity [m/s], steer [rad]
    /reset     (std_msgs/Empty)        reset vehicle to its initial pose
Publishes:
    /veh_state  (thr_sim_msgs/VehState) x, y, yaw
    /veh_actual (thr_sim_msgs/VehCtrl)  actual velocity and steer (PI loop outputs)
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty

from thr_sim_msgs.msg import VehCtrl, VehState
from thr_sim.bicycle_model import BicycleModel


class VehicleSimNode(Node):
    def __init__(self):
        super().__init__('vehicle_sim')
        p = self.declare_parameter
        self.dt = p('dt', 0.01).value                      # integration step [s]
        self.publish_rate = p('publish_rate', 50.0).value  # /veh_state rate [Hz]
        self.cmd_timeout = p('cmd_timeout', 0.5).value     # stop if no cmd [s]
        self.init_pose = (p('init_x', 0.0).value,
                          p('init_y', 0.0).value,
                          p('init_yaw', 0.0).value)
        self.model = BicycleModel(
            wheelbase=p('wheelbase', 2.5).value,
            max_steer=p('max_steer', 0.6).value,
            max_steer_rate=p('max_steer_rate', 1.5).value,
            max_speed=p('max_speed', 15.0).value,
            max_accel=p('max_accel', 3.0).value,
            max_decel=p('max_decel', 6.0).value,
            speed_kp=p('speed_kp', 4.8).value,
            speed_ki=p('speed_ki', 9.0).value,
            steer_kp=p('steer_kp', 16.0).value,
            steer_ki=p('steer_ki', 100.0).value,
            speed_noise_std=p('speed_noise_std', 0.1).value,
            steer_noise_std=p('steer_noise_std', 0.005).value,
            noise_tau=p('noise_tau', 0.2).value,
            seed=None if (seed := p('noise_seed', -1).value) < 0 else seed,
        )

        self.cmd_v = 0.0
        self.cmd_steer = 0.0
        self.last_cmd_time = None
        self.reset()

        self.create_subscription(VehCtrl, '/veh_ctrl', self.on_ctrl, 10)
        self.create_subscription(Empty, '/reset', self.on_reset, 10)
        self.state_pub = self.create_publisher(VehState, '/veh_state', 10)
        self.actual_pub = self.create_publisher(VehCtrl, '/veh_actual', 10)

        self.create_timer(self.dt, self.on_step)
        self.create_timer(1.0 / self.publish_rate, self.publish_state)
        self.get_logger().info(
            f'Vehicle sim ready (L={self.model.L} m, dt={self.dt} s, '
            f'init={self.init_pose})')

    def reset(self):
        self.model.reset(*self.init_pose)
        self.cmd_v = 0.0
        self.cmd_steer = 0.0
        self.last_cmd_time = None

    def on_reset(self, _msg):
        self.reset()
        self.get_logger().info('Reset received: vehicle back at initial pose')
        self.publish_state()

    def on_ctrl(self, msg: VehCtrl):
        self.cmd_v = msg.velocity
        self.cmd_steer = msg.steer
        self.last_cmd_time = self.get_clock().now()

    def on_step(self):
        # Safety: hold zero speed if commands stop arriving.
        if self.last_cmd_time is None or (
                self.get_clock().now() - self.last_cmd_time
        ).nanoseconds * 1e-9 > self.cmd_timeout:
            v, steer = 0.0, self.model.steer.value   # hold steer (without noise)
        else:
            v, steer = self.cmd_v, self.cmd_steer
        self.model.step(v, steer, self.dt)

    def publish_state(self):
        s = self.model.state
        msg = VehState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.x, msg.y, msg.yaw = s.x, s.y, s.yaw
        self.state_pub.publish(msg)
        act = VehCtrl()
        act.header = msg.header
        act.velocity, act.steer = s.v, s.steer
        self.actual_pub.publish(act)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleSimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
