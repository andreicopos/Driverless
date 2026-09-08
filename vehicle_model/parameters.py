from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class VehicleParameters:
    """Longitudinal vehicle parameters.

    Values in DEFAULT_TEST_PARAMETERS are intentionally generic test assumptions,
    not measured ARTTU data.
    """

    mass_kg: float
    wheel_radius_m: float
    gear_ratio: float
    drivetrain_efficiency: float
    rolling_resistance_coeff: float
    air_density_kg_m3: float
    drag_coeff: float
    frontal_area_m2: float
    road_grade_rad: float = 0.0

    def validate(self) -> None:
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


# TEMPORARY_TEST_ONLY: generic values used only to exercise the model.
DEFAULT_TEST_PARAMETERS = VehicleParameters(
    mass_kg=250.0,
    wheel_radius_m=0.23,
    gear_ratio=4.5,
    drivetrain_efficiency=0.95,
    rolling_resistance_coeff=0.015,
    air_density_kg_m3=1.225,
    drag_coeff=0.9,
    frontal_area_m2=1.2,
)
