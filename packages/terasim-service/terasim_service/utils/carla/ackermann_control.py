import math
from dataclasses import dataclass


@dataclass(frozen=True)
class AckermannTuning:
    wheel_base: float = 2.8
    max_steer_rad: float = 0.6
    max_steer_rate_rad_s: float = 0.6
    kp_speed: float = 0.8
    kp_position: float = 0.15
    max_accel: float = 3.0
    max_decel: float = 6.0


@dataclass(frozen=True)
class AckermannControllerTuning:
    """CARLA's internal cascaded speed/acceleration PID settings."""

    speed_kp: float = 0.15
    speed_ki: float = 0.0
    speed_kd: float = 0.25
    accel_kp: float = 0.01
    accel_ki: float = 0.0
    accel_kd: float = 0.01


@dataclass(frozen=True)
class AckermannControlValues:
    steer: float
    raw_steer: float
    clamped_steer: float
    lookahead_local_x: float
    lookahead_local_y: float
    speed: float
    acceleration: float
    jerk: float
    position_error: float
    longitudinal_error: float


def clamp(value, lower, upper):
    return min(upper, max(lower, value))


def horizontal_speed(velocity):
    return math.hypot(float(getattr(velocity, "x", 0.0)), float(getattr(velocity, "y", 0.0)))


def world_to_vehicle_2d(origin_x, origin_y, yaw_degrees, point_x, point_y):
    dx = float(point_x) - float(origin_x)
    dy = float(point_y) - float(origin_y)
    yaw = math.radians(float(yaw_degrees))
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        cos_yaw * dx + sin_yaw * dy,
        -sin_yaw * dx + cos_yaw * dy,
    )


def rate_limit(value, previous_value, max_rate, dt):
    if previous_value is None or max_rate <= 0.0 or dt <= 0.0:
        return value
    max_delta = max_rate * dt
    return clamp(value, previous_value - max_delta, previous_value + max_delta)


def compute_ackermann_control_values(
    *,
    current_x,
    current_y,
    yaw_degrees,
    current_speed,
    desired_x,
    desired_y,
    lookahead_x,
    lookahead_y,
    desired_speed,
    previous_steer=None,
    dt=0.1,
    tuning=AckermannTuning(),
):
    desired_speed = max(0.0, float(desired_speed))
    current_speed = max(0.0, float(current_speed))

    lookahead_local_x, lookahead_local_y = world_to_vehicle_2d(
        current_x, current_y, yaw_degrees, lookahead_x, lookahead_y
    )
    desired_local_x, desired_local_y = world_to_vehicle_2d(
        current_x, current_y, yaw_degrees, desired_x, desired_y
    )

    lookahead_distance = max(math.hypot(lookahead_local_x, lookahead_local_y), 0.1)
    alpha = math.atan2(lookahead_local_y, lookahead_local_x)
    curvature = 2.0 * math.sin(alpha) / lookahead_distance

    raw_steer = math.atan(float(tuning.wheel_base) * curvature)
    clamped_steer = clamp(raw_steer, -tuning.max_steer_rad, tuning.max_steer_rad)
    steer = rate_limit(clamped_steer, previous_steer, tuning.max_steer_rate_rad_s, dt)

    position_error = math.hypot(desired_local_x, desired_local_y)
    acceleration = (
        tuning.kp_speed * (desired_speed - current_speed) + tuning.kp_position * desired_local_x
    )
    acceleration = clamp(acceleration, -tuning.max_decel, tuning.max_accel)

    return AckermannControlValues(
        steer=steer,
        raw_steer=raw_steer,
        clamped_steer=clamped_steer,
        lookahead_local_x=lookahead_local_x,
        lookahead_local_y=lookahead_local_y,
        speed=desired_speed,
        acceleration=acceleration,
        jerk=0.0,
        position_error=position_error,
        longitudinal_error=desired_local_x,
    )
