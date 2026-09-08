import math
from dataclasses import dataclass

from .arttu_parameters import ARTTUParameters
from .drivetrain import interpolate_torque_speed


@dataclass(frozen=True)
class DrivetrainOutput:
    motor_rpm_left: float
    motor_rpm_right: float
    torque_map_left_nm: float
    torque_map_right_nm: float
    torque_available_left_nm: float
    torque_available_right_nm: float
    torque_command_left_nm: float
    torque_command_right_nm: float
    mechanical_power_left_w: float
    mechanical_power_right_w: float
    mechanical_power_total_w: float
    battery_power_left_w: float
    battery_power_right_w: float
    battery_power_total_w: float
    per_inverter_power_limited: bool
    total_battery_power_limited: bool

    @property
    def power_limited(self) -> bool:
        return self.per_inverter_power_limited or self.total_battery_power_limited


def motor_rpm_from_vehicle_speed(
    vehicle_speed_mps: float,
    gear_ratio: float,
    wheel_radius_m: float,
) -> float:
    if vehicle_speed_mps < 0:
        raise ValueError("vehicle_speed_mps must be >= 0")
    if gear_ratio <= 0 or wheel_radius_m <= 0:
        raise ValueError("gear_ratio and wheel_radius_m must be > 0")
    wheel_omega = vehicle_speed_mps / wheel_radius_m
    motor_omega = wheel_omega * gear_ratio
    return motor_omega * 60.0 / (2.0 * math.pi)


def _omega_from_rpm(rpm: float) -> float:
    return rpm * 2.0 * math.pi / 60.0


def _mechanical_power(torque_nm: float, rpm: float) -> float:
    """Positive mechanical power only; regen is reserved for the later brake model."""
    return max(0.0, torque_nm * _omega_from_rpm(rpm))


def _limit_per_inverter_power(
    torque_left_nm: float,
    torque_right_nm: float,
    motor_rpm: float,
    params: ARTTUParameters,
) -> tuple[float, float, bool]:
    """Apply the MST400-133 43 kW/controller continuous power ceiling.

    The manual documents this as a controller power rating. We conservatively treat
    it as an electrical-side ceiling for the per-inverter power limiter. Because the
    project-level combined battery limit is only 40 kW, this limiter is normally
    inactive for symmetric operation but remains important for future asymmetric
    torque-vectoring commands.
    """
    omega = _omega_from_rpm(motor_rpm)
    if omega <= 0.0:
        return torque_left_nm, torque_right_nm, False

    max_mech_per_inverter = params.inverter_continuous_power_w * params.converter_efficiency
    left_limit = min(torque_left_nm, max_mech_per_inverter / omega)
    right_limit = min(torque_right_nm, max_mech_per_inverter / omega)
    limited = left_limit < torque_left_nm or right_limit < torque_right_nm
    return left_limit, right_limit, limited


def _limit_total_battery_power(
    torque_left_nm: float,
    torque_right_nm: float,
    motor_rpm: float,
    params: ARTTUParameters,
) -> tuple[float, float, bool]:
    """Apply the 40 kW combined battery-side power restriction.

    The workbook supplies 90% converter efficiency. Therefore the available positive
    mechanical power for the current working model is:

        P_mech,max = P_battery,max * eta_converter

    Left/right positive torque requests are scaled proportionally so the requested
    torque split is preserved.
    """
    omega = _omega_from_rpm(motor_rpm)
    requested_mech = _mechanical_power(torque_left_nm, motor_rpm) + _mechanical_power(torque_right_nm, motor_rpm)
    max_mech_total = params.total_battery_power_limit_w * params.converter_efficiency

    if requested_mech <= 0.0 or requested_mech <= max_mech_total:
        return torque_left_nm, torque_right_nm, False

    scale = max_mech_total / requested_mech
    return torque_left_nm * scale, torque_right_nm * scale, True


def command_from_accelerator(
    vehicle_speed_mps: float,
    normalized_command: float,
    params: ARTTUParameters,
) -> DrivetrainOutput:
    """Convert a normalized [0,1] accelerator request into two motor torques.

    Phase 2B first-version architecture:
      1. torque is obtained from the supplied motor torque-speed map;
      2. torque is split equally left/right;
      3. each inverter's continuous power ceiling is applied;
      4. the 40 kW combined battery-side ceiling is applied.

    No detailed current-control or electromagnetic motor model is introduced here.
    """
    if not 0.0 <= normalized_command <= 1.0:
        raise ValueError("normalized_command must be in [0, 1]")
    if params.wheel_radius_m is None:
        raise ValueError("wheel_radius_m is required")

    rpm = motor_rpm_from_vehicle_speed(
        vehicle_speed_mps,
        params.gear_ratio,
        params.wheel_radius_m,
    )

    if rpm > params.motor_map_max_rpm:
        torque_map = 0.0
    else:
        torque_map = interpolate_torque_speed(
            rpm,
            params.motor_speed_rpm,
            params.motor_torque_nm,
        )

    requested_left = normalized_command * torque_map
    requested_right = normalized_command * torque_map

    limited_left, limited_right, per_inv_limited = _limit_per_inverter_power(
        requested_left,
        requested_right,
        rpm,
        params,
    )
    applied_left, applied_right, total_limited = _limit_total_battery_power(
        limited_left,
        limited_right,
        rpm,
        params,
    )

    p_mech_left = _mechanical_power(applied_left, rpm)
    p_mech_right = _mechanical_power(applied_right, rpm)
    p_batt_left = p_mech_left / params.converter_efficiency
    p_batt_right = p_mech_right / params.converter_efficiency

    return DrivetrainOutput(
        motor_rpm_left=rpm,
        motor_rpm_right=rpm,
        torque_map_left_nm=torque_map,
        torque_map_right_nm=torque_map,
        torque_available_left_nm=limited_left,
        torque_available_right_nm=limited_right,
        torque_command_left_nm=applied_left,
        torque_command_right_nm=applied_right,
        mechanical_power_left_w=p_mech_left,
        mechanical_power_right_w=p_mech_right,
        mechanical_power_total_w=p_mech_left + p_mech_right,
        battery_power_left_w=p_batt_left,
        battery_power_right_w=p_batt_right,
        battery_power_total_w=p_batt_left + p_batt_right,
        per_inverter_power_limited=per_inv_limited,
        total_battery_power_limited=total_limited,
    )
