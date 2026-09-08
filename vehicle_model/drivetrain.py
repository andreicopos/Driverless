from dataclasses import dataclass
from bisect import bisect_right

from .parameters import VehicleParameters


@dataclass(frozen=True)
class MotorCommand:
    """Motor torque requests at the motor shaft, in N*m."""

    torque_left_nm: float
    torque_right_nm: float


@dataclass(frozen=True)
class MotorLimits:
    """Simplified limits that can be applied to each motor."""

    max_torque_nm: float | None = None
    max_motor_rpm: float | None = None


def saturate_torque(torque_nm: float, max_torque_nm: float | None) -> float:
    if max_torque_nm is None:
        return torque_nm
    if max_torque_nm < 0:
        raise ValueError("max_torque_nm must be >= 0")
    return max(-max_torque_nm, min(max_torque_nm, torque_nm))


def interpolate_torque_speed(speed_rpm: float, speed_rpm_map, torque_nm_map) -> float:
    """Piecewise-linear torque-speed lookup; no extrapolation beyond the supplied map."""
    if len(speed_rpm_map) != len(torque_nm_map) or len(speed_rpm_map) < 2:
        raise ValueError("Motor torque-speed map must contain matching arrays with >=2 points")
    if speed_rpm < speed_rpm_map[0] or speed_rpm > speed_rpm_map[-1]:
        raise ValueError(
            f"Motor speed {speed_rpm:.1f} rpm is outside supplied map "
            f"[{speed_rpm_map[0]:.1f}, {speed_rpm_map[-1]:.1f}] rpm"
        )

    idx = bisect_right(speed_rpm_map, speed_rpm) - 1
    if idx >= len(speed_rpm_map) - 1:
        return float(torque_nm_map[-1])

    x0, x1 = speed_rpm_map[idx], speed_rpm_map[idx + 1]
    y0, y1 = torque_nm_map[idx], torque_nm_map[idx + 1]
    if x1 == x0:
        return float(y0)
    fraction = (speed_rpm - x0) / (x1 - x0)
    return float(y0 + fraction * (y1 - y0))


def motor_torque_to_wheel_torque(
    motor_torque_nm: float,
    params: VehicleParameters,
) -> float:
    return motor_torque_nm * params.gear_ratio * params.drivetrain_efficiency


def wheel_torque_to_force(wheel_torque_nm: float, wheel_radius_m: float) -> float:
    if wheel_radius_m <= 0:
        raise ValueError("wheel_radius_m must be > 0")
    return wheel_torque_nm / wheel_radius_m


def motor_torques_to_forces(
    command: MotorCommand,
    params: VehicleParameters,
    limits: MotorLimits | None = None,
) -> tuple[float, float]:
    limits = limits or MotorLimits()
    left = saturate_torque(command.torque_left_nm, limits.max_torque_nm)
    right = saturate_torque(command.torque_right_nm, limits.max_torque_nm)

    left_wheel = motor_torque_to_wheel_torque(left, params)
    right_wheel = motor_torque_to_wheel_torque(right, params)

    return (
        wheel_torque_to_force(left_wheel, params.wheel_radius_m),
        wheel_torque_to_force(right_wheel, params.wheel_radius_m),
    )
