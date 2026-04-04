#!/usr/bin/env python3

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class LimitConfig:
    max_planar_speed: float
    max_angular_speed: float
    max_planar_acceleration: float
    max_angular_acceleration: float
    max_wheel_speed: float = 0.0
    wheel_count: int = 0
    wheel_offset: float = 0.0
    robot_radius: float = 0.0
    wheel_radius: float = 0.0


def omni_wheel_angles(wheel_count: int, wheel_offset: float) -> list[float]:
    return [
        wheel_offset + index * (2.0 * math.pi / wheel_count)
        for index in range(wheel_count)
    ]


def body_twist_to_omni_wheel_speeds(
    vx: float,
    vy: float,
    wz: float,
    wheel_count: int,
    wheel_offset: float,
    robot_radius: float,
    wheel_radius: float,
) -> list[float]:
    if wheel_count < 1 or wheel_radius <= 0.0:
        return []
    return [
        (-math.sin(alpha) * vx + math.cos(alpha) * vy + robot_radius * wz)
        / wheel_radius
        for alpha in omni_wheel_angles(wheel_count, wheel_offset)
    ]


def omni_wheel_speeds_to_body_twist(
    wheel_speeds: list[float],
    wheel_count: int,
    wheel_offset: float,
    robot_radius: float,
    wheel_radius: float,
) -> tuple[float, float, float]:
    if wheel_count < 1 or wheel_radius <= 0.0 or robot_radius <= 0.0:
        return 0.0, 0.0, 0.0

    vx = 0.0
    vy = 0.0
    wz = 0.0
    for alpha, wheel_speed in zip(
        omni_wheel_angles(wheel_count, wheel_offset), wheel_speeds,
    ):
        rim_speed = wheel_speed * wheel_radius
        vx += -math.sin(alpha) * rim_speed
        vy += math.cos(alpha) * rim_speed
        wz += rim_speed

    vx *= 2.0 / wheel_count
    vy *= 2.0 / wheel_count
    wz /= wheel_count * robot_radius
    return vx, vy, wz


def clamp_body_twist(
    vx: float,
    vy: float,
    wz: float,
    max_planar_speed: float,
    max_angular_speed: float,
) -> tuple[float, float, float]:
    planar_speed = math.hypot(vx, vy)
    if planar_speed > max_planar_speed > 0.0:
        scale = max_planar_speed / planar_speed
        vx *= scale
        vy *= scale
    if max_angular_speed > 0.0:
        wz = max(-max_angular_speed, min(max_angular_speed, wz))
    return vx, vy, wz


def apply_acceleration_limits(
    current_twist: tuple[float, float, float],
    target_twist: tuple[float, float, float],
    dt: float,
    max_planar_acceleration: float,
    max_angular_acceleration: float,
) -> tuple[float, float, float]:
    current_vx, current_vy, current_wz = current_twist
    target_vx, target_vy, target_wz = target_twist

    delta_vx = target_vx - current_vx
    delta_vy = target_vy - current_vy
    planar_delta = math.hypot(delta_vx, delta_vy)
    max_planar_delta = max_planar_acceleration * dt
    if planar_delta > max_planar_delta > 0.0:
        scale = max_planar_delta / planar_delta
        delta_vx *= scale
        delta_vy *= scale

    delta_wz = target_wz - current_wz
    max_wz_delta = max_angular_acceleration * dt
    if abs(delta_wz) > max_wz_delta > 0.0:
        delta_wz = math.copysign(max_wz_delta, delta_wz)

    return current_vx + delta_vx, current_vy + delta_vy, current_wz + delta_wz


def apply_omni_wheel_speed_limit(
    vx: float,
    vy: float,
    wz: float,
    wheel_count: int,
    wheel_offset: float,
    robot_radius: float,
    wheel_radius: float,
    max_wheel_speed: float,
) -> tuple[float, float, float]:
    if max_wheel_speed <= 0.0 or wheel_count < 1 or wheel_radius <= 0.0:
        return vx, vy, wz

    wheel_speeds = body_twist_to_omni_wheel_speeds(
        vx,
        vy,
        wz,
        wheel_count,
        wheel_offset,
        robot_radius,
        wheel_radius,
    )
    peak_speed = max(abs(wheel_speed) for wheel_speed in wheel_speeds)

    if peak_speed > max_wheel_speed:
        scale = max_wheel_speed / peak_speed
        return vx * scale, vy * scale, wz * scale
    return vx, vy, wz


def limit_body_twist(
    current_twist: tuple[float, float, float],
    target_twist: tuple[float, float, float],
    dt: float,
    config: LimitConfig,
) -> tuple[float, float, float]:
    limited_twist = clamp_body_twist(
        target_twist[0],
        target_twist[1],
        target_twist[2],
        config.max_planar_speed,
        config.max_angular_speed,
    )
    limited_twist = apply_acceleration_limits(
        current_twist,
        limited_twist,
        dt,
        config.max_planar_acceleration,
        config.max_angular_acceleration,
    )
    limited_twist = apply_omni_wheel_speed_limit(
        limited_twist[0],
        limited_twist[1],
        limited_twist[2],
        config.wheel_count,
        config.wheel_offset,
        config.robot_radius,
        config.wheel_radius,
        config.max_wheel_speed,
    )
    return clamp_body_twist(
        limited_twist[0],
        limited_twist[1],
        limited_twist[2],
        config.max_planar_speed,
        config.max_angular_speed,
    )


def yaw_to_quaternion_components(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def official_base_action_to_twist(
    action: dict[str, float],
) -> tuple[float, float, float]:
    return (
        float(action.get("x.vel", 0.0)),
        float(action.get("y.vel", 0.0)),
        math.radians(float(action.get("theta.vel", 0.0))),
    )


def odom_twist_to_official_base_observation(
    vx: float,
    vy: float,
    wz: float,
) -> dict[str, float]:
    return {
        "x.vel": float(vx),
        "y.vel": float(vy),
        "theta.vel": math.degrees(float(wz)),
    }


def official_position_to_radians(
    value: float,
    lower: float,
    upper: float,
    use_degrees: bool,
    zero_to_hundred: bool = False,
) -> float:
    if zero_to_hundred:
        normalized = max(0.0, min(100.0, float(value))) / 100.0
        return lower + normalized * (upper - lower)
    if use_degrees:
        radians = math.radians(float(value))
        return max(lower, min(upper, radians))
    normalized = (max(-100.0, min(100.0, float(value))) + 100.0) / 200.0
    return lower + normalized * (upper - lower)


def radians_to_official_position(
    value: float,
    lower: float,
    upper: float,
    use_degrees: bool,
    zero_to_hundred: bool = False,
) -> float:
    clipped = max(lower, min(upper, float(value)))
    if zero_to_hundred:
        if upper <= lower:
            return 0.0
        return (clipped - lower) / (upper - lower) * 100.0
    if use_degrees:
        return math.degrees(clipped)
    if upper <= lower:
        return 0.0
    return ((clipped - lower) / (upper - lower)) * 200.0 - 100.0


def extract_planar_joint_state(
    joint_names: list[str],
    positions: list[float],
    velocities: list[float],
) -> tuple[float, float, float, float, float, float] | None:
    try:
        x_index = joint_names.index("root_x_axis_joint")
        y_index = joint_names.index("root_y_axis_joint")
        yaw_index = joint_names.index("root_z_rotation_joint")
    except ValueError:
        return None

    if len(positions) <= max(x_index, y_index, yaw_index):
        return None

    x = positions[x_index]
    y = positions[y_index]
    yaw = positions[yaw_index]
    vx = 0.0
    vy = 0.0
    wz = 0.0
    if len(velocities) > max(x_index, y_index, yaw_index):
        vx = velocities[x_index]
        vy = velocities[y_index]
        wz = velocities[yaw_index]
    return x, y, yaw, vx, vy, wz
