from dataclasses import replace
import math

from .state import VehicleState
from .command import VehicleCommand
from .parameters import VehicleParameters


class KinematicBicycle:

    def __init__(self, parameters: VehicleParameters):
        self.parameters = parameters

    def step(
        self,
        state: VehicleState,
        command: VehicleCommand,
        dt: float,
    ) -> VehicleState:

        # Steering saturation
        delta = max(
            -self.parameters.max_steering_angle,
            min(
                command.steering_angle,
                self.parameters.max_steering_angle,
            ),
        )

        # Current vehicle speed
        v = max(0.0, state.velocity)

        # Kinematic bicycle equations
        x_dot = v * math.cos(state.yaw)
        y_dot = v * math.sin(state.yaw)

        yaw_dot = (
            v
            / self.parameters.wheelbase
            * math.tan(delta)
        )

        # Euler integration
        x = state.x + x_dot * dt
        y = state.y + y_dot * dt
        yaw = state.yaw + yaw_dot * dt

        velocity = max(
            0.0,
            v + command.longitudinal_command * dt
        )

        return VehicleState(
            x=x,
            y=y,
            yaw=yaw,
            velocity=velocity,
        )