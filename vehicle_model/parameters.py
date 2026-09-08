from dataclasses import dataclass
import math


@dataclass(frozen=True)
class VehicleParameters:
    """Shared vehicle parameters for kinematic and longitudinal models.

    Kinematic parameters are kept compatible with the Phase 2A API while the
    longitudinal parameters support the Phase 2B vehicle model.

    DEFAULT_TEST_PARAMETERS uses generic assumptions and is not measured ARTTU data.
    """

    # Phase 2A kinematic parameters
    wheelbase: float = 1.6
    max_steering_angle: float = math.radians(30.0)

    # Phase 2B longitudinal parameters
    mass_kg: float = 250.0
    wheel_radius_m: float = 0.23
    gear_ratio: float = 4.5
    drivetrain_efficiency: float = 0.95
    rolling_resistance_coeff: float = 0.015
    air_density_kg_m3: float = 1.225
    drag_coeff: float = 0.9
    frontal_area_m2: float = 1.2
    road_grade_rad: float = 0.0

    def validate(self) -> None:
        if self.wheelbase <= 0:
            raise ValueError("wheelbase must be > 0")
        if self.max_steering_angle < 0:
            raise ValueError("max_steering_angle must be >= 0")
        if self.mass_kg <= 0:
            raise ValueError("mass_kg must be > 0")
        if self.wheel_radius_m <= 0:
            raise ValueError("wheel_radius_m must be > 0")
        if self.gear_ratio <= 0:
            raise ValueError("gear_ratio must be > 0")
        if not 0 < self.drivetrain_efficiency <= 1:
            raise ValueError("drivetrain_efficiency must be in (0, 1]")
        if self.rolling_resistance_coeff < 0:
            raise ValueError("rolling_resistance_coeff must be >= 0")
        if self.air_density_kg_m3 < 0:
            raise ValueError("air_density_kg_m3 must be >= 0")
        if self.drag_coeff < 0:
            raise ValueError("drag_coeff must be >= 0")
        if self.frontal_area_m2 < 0:
            raise ValueError("frontal_area_m2 must be >= 0")


# Generic values used only to exercise the model; not ARTTU measurements.
DEFAULT_TEST_PARAMETERS = VehicleParameters()
