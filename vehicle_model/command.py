from dataclasses import dataclass


@dataclass
class VehicleCommand:
    steering_angle: float = 0.0
    longitudinal_command: float = 0.0