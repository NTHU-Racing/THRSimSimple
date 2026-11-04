"""Kinematic bicycle model (rear-axle reference point), ROS-independent."""
import math
import random
from dataclasses import dataclass


def wrap_angle(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class PIActuator:
    """PI loop driving an integrating actuator.

        rate   = clamp(kp * e + ki * integral(e), -max_rate_neg, max_rate_pos)
        value += rate * dt,  value clamped to [-max_value, max_value]

    With an integrator plant the closed loop is s^2 + kp*s + ki, so
    kp = 2*zeta*wn and ki = wn^2. The integral is only updated while the output
    is not saturated (conditional-integration anti-windup).
    """

    def __init__(self, kp, ki, max_rate_pos, max_rate_neg, max_value):
        self.kp, self.ki = kp, ki
        self.max_rate_pos, self.max_rate_neg = max_rate_pos, max_rate_neg
        self.max_value = max_value
        self.value = 0.0
        self.integral = 0.0

    def reset(self, value=0.0):
        self.value = value
        self.integral = 0.0

    def step(self, cmd: float, dt: float) -> float:
        cmd = clamp(cmd, -self.max_value, self.max_value)
        e = cmd - self.value
        integral = self.integral + e * dt
        u = self.kp * e + self.ki * integral
        rate = clamp(u, -self.max_rate_neg, self.max_rate_pos)
        if rate == u:
            self.integral = integral
        self.value = clamp(self.value + rate * dt, -self.max_value, self.max_value)
        return self.value


class ColoredNoise:
    """Zero-mean Gaussian noise with correlation time `tau` (Ornstein-Uhlenbeck):
        n += -n * dt / tau + std * sqrt(2 * dt / tau) * N(0, 1)
    Its stationary standard deviation is `std`; std = 0 disables it."""

    def __init__(self, std, tau, rng):
        self.std, self.tau, self.rng = std, tau, rng
        self.value = 0.0

    def reset(self):
        self.value = 0.0

    def step(self, dt):
        if self.std <= 0.0:
            return 0.0
        a = dt / self.tau
        self.value += -self.value * a + self.std * math.sqrt(2.0 * a) * self.rng.gauss(0.0, 1.0)
        return self.value


@dataclass
class BicycleState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0       # actual speed (PI speed loop output + noise)
    steer: float = 0.0   # actual steering angle (PI steer loop output + noise)


class BicycleModel:
    """
    Kinematic bicycle model:
        x'   = v * cos(yaw)
        y'   = v * sin(yaw)
        yaw' = v / L * tan(steer)

    Actual speed and steering angle follow their commands through PI loops
    (see PIActuator): the speed loop outputs acceleration limited to
    max_accel / max_decel, the steering loop outputs steering rate limited to
    max_steer_rate. The kinematics are integrated with RK4 each step.

    Optional actuator noise: colored Gaussian noise (std, correlation time
    noise_tau) is added to the PI loop outputs. The PI loops do not see it, so
    it acts as a disturbance. Speed noise fades out below 1 m/s so a stopped
    vehicle does not creep. `seed` makes the noise repeatable; it is re-seeded
    on every reset, so each run gets the same noise sequence.
    """

    def __init__(self, wheelbase=2.5, max_steer=0.6, max_steer_rate=1.5,
                 max_speed=15.0, max_accel=3.0, max_decel=6.0,
                 speed_kp=4.8, speed_ki=9.0, steer_kp=16.0, steer_ki=100.0,
                 speed_noise_std=0.0, steer_noise_std=0.0, noise_tau=0.2, seed=None):
        self.L = wheelbase
        self.max_steer = max_steer
        self.speed = PIActuator(speed_kp, speed_ki, max_accel, max_decel, max_speed)
        self.steer = PIActuator(steer_kp, steer_ki, max_steer_rate, max_steer_rate,
                                max_steer)
        self.seed = seed
        self.rng = random.Random(seed)
        self.speed_noise = ColoredNoise(speed_noise_std, noise_tau, self.rng)
        self.steer_noise = ColoredNoise(steer_noise_std, noise_tau, self.rng)
        self.state = BicycleState()

    def reset(self, x=0.0, y=0.0, yaw=0.0, v=0.0, steer=0.0):
        self.speed.reset(v)
        self.steer.reset(steer)
        if self.seed is not None:
            self.rng.seed(self.seed)
        self.speed_noise.reset()
        self.steer_noise.reset()
        self.state = BicycleState(x=x, y=y, yaw=wrap_angle(yaw), v=v, steer=steer)

    def step(self, v_cmd: float, steer_cmd: float, dt: float) -> BicycleState:
        s = self.state
        v_pi = self.speed.step(v_cmd, dt)
        steer_pi = self.steer.step(steer_cmd, dt)
        s.v = v_pi + min(1.0, abs(v_pi)) * self.speed_noise.step(dt)
        s.steer = clamp(steer_pi + self.steer_noise.step(dt), -self.max_steer, self.max_steer)

        # RK4 integration of the kinematics with constant v, steer over dt.
        v, L, tan_d = s.v, self.L, math.tan(s.steer)

        def f(yaw):
            return v * math.cos(yaw), v * math.sin(yaw), v / L * tan_d

        k1 = f(s.yaw)
        k2 = f(s.yaw + 0.5 * dt * k1[2])
        k3 = f(s.yaw + 0.5 * dt * k2[2])
        k4 = f(s.yaw + dt * k3[2])
        s.x += dt / 6.0 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
        s.y += dt / 6.0 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
        s.yaw = wrap_angle(s.yaw + dt / 6.0 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]))
        return s
