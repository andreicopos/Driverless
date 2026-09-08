from .state import VehicleState
from .command import VehicleCommand
from .kinematic_bicycle import KinematicBicycle


class Simulator:

    def __init__(
        self,
        model: KinematicBicycle,
        state: VehicleState,
        dt: float,
    ):
        self.model = model
        self.state = state
        self.dt = dt

    def step(self, command: VehicleCommand) -> VehicleState:
        self.state = self.model.step(
            self.state,
            command,
            self.dt,
        )

        return self.state