from dataclasses import dataclass

from .drivetrain import MotorCommand, MotorLimits
from .longitudinal_dynamics import LongitudinalOutput, LongitudinalState, model_step
from .parameters import VehicleParameters


@dataclass(frozen=True)
class SimulationLog:
    time_s: list[float]
    position_m: list[float]
    velocity_mps: list[float]
    acceleration_mps2: list[float]
    force_n: list[float]


def simulate(
    initial_state: LongitudinalState,
    commands: list[MotorCommand],
    dt_s: float,
    params: VehicleParameters,
    limits: MotorLimits | None = None,
) -> SimulationLog:
    state = initial_state
    time = [0.0]
    positions = [state.position_m]
    velocities = [state.velocity_mps]
    accelerations = [0.0]
    forces = [0.0]

    for i, command in enumerate(commands, start=1):
        state, output = model_step(state, command, dt_s, params, limits)
        time.append(i * dt_s)
        positions.append(state.position_m)
        velocities.append(state.velocity_mps)
        accelerations.append(output.acceleration_mps2)
        forces.append(output.forces.net_force_n)

    return SimulationLog(
        time_s=time,
        position_m=positions,
        velocity_mps=velocities,
        acceleration_mps2=accelerations,
        force_n=forces,
    )
