import math
from dataclasses import dataclass

from .drivetrain import MotorCommand, MotorLimits, motor_torques_to_forces
from .parameters import VehicleParameters


@dataclass(frozen=True)
class LongitudinalForces:
    tractive_left_n: float
    tractive_right_n: float
    tractive_total_n: float
    rolling_resistance_n: float
    aerodynamic_drag_n: float
    grade_force_n: float
    net_force_n: float


@dataclass(frozen=True)
class LongitudinalState:
    position_m: float
    velocity_mps: float


@dataclass(frozen=True)
class LongitudinalOutput:
    acceleration_mps2: float
    forces: LongitudinalForces


def rolling_resistance_force(params: VehicleParameters) -> float:
    return (
        params.rolling_resistance_coeff
        * params.mass_kg
        * 9.80665
        * math.cos(params.road_grade_rad)
    )


def aerodynamic_drag_force(velocity_mps: float, params: VehicleParameters) -> float:
    return (
        0.5
        * params.air_density_kg_m3
        * params.drag_coeff
        * params.frontal_area_m2
        * velocity_mps**2
    )


def grade_force(params: VehicleParameters) -> float:
    return params.mass_kg * 9.80665 * math.sin(params.road_grade_rad)


def compute_forces(
    velocity_mps: float,
    command: MotorCommand,
    params: VehicleParameters,
    limits: MotorLimits | None = None,
) -> LongitudinalForces:
    params.validate()
    f_left, f_right = motor_torques_to_forces(command, params, limits)
    f_rr = rolling_resistance_force(params)
    f_aero = aerodynamic_drag_force(velocity_mps, params)
    f_grade = grade_force(params)
    f_total = f_left + f_right
    f_net = f_total - f_rr - f_aero - f_grade

    return LongitudinalForces(
        tractive_left_n=f_left,
        tractive_right_n=f_right,
        tractive_total_n=f_total,
        rolling_resistance_n=f_rr,
        aerodynamic_drag_n=f_aero,
        grade_force_n=f_grade,
        net_force_n=f_net,
    )


def acceleration_from_forces(net_force_n: float, mass_kg: float) -> float:
    if mass_kg <= 0:
        raise ValueError("mass_kg must be > 0")
    return net_force_n / mass_kg


def model_step(
    state: LongitudinalState,
    command: MotorCommand,
    dt_s: float,
    params: VehicleParameters,
    limits: MotorLimits | None = None,
) -> tuple[LongitudinalState, LongitudinalOutput]:
    if dt_s <= 0:
        raise ValueError("dt_s must be > 0")

    forces = compute_forces(state.velocity_mps, command, params, limits)
    acceleration = acceleration_from_forces(forces.net_force_n, params.mass_kg)

    next_velocity = max(0.0, state.velocity_mps + acceleration * dt_s)
    next_position = state.position_m + state.velocity_mps * dt_s + 0.5 * acceleration * dt_s**2

    return (
        LongitudinalState(position_m=next_position, velocity_mps=next_velocity),
        LongitudinalOutput(acceleration_mps2=acceleration, forces=forces),
    )
