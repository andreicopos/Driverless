from dataclasses import dataclass
import math

from .parameters import VehicleParameters


# Values directly supported by the supplied ARTTU files/user data.
ARTTU_MASS_KG = 239.0
ARTTU_GEAR_RATIO = 5.0
ARTTU_INVERTER_CONTINUOUS_POWER_W = 43_000.0
ARTTU_TOTAL_BATTERY_POWER_LIMIT_W = 40_000.0

# The Powertrain workbook's Converter sheet lists 90% efficiency.
# This is treated as a model input from the workbook, not as a motor datasheet value.
ARTTU_CONVERTER_EFFICIENCY = 0.90

# Chassis -> Aero sheet.
ARTTU_AIR_DENSITY_KG_M3 = 1.225
ARTTU_FRONTAL_AREA_M2 = 1.2
ARTTU_CD_MAGNITUDE = 1.8  # Workbook stores CD=-1.8 because of its force-sign convention.
ARTTU_CL_MAGNITUDE = 3.0  # Not used in Phase 2B longitudinal model.

# Tire file name in the supplied chassis workbook: Hoosier_20p5x7_13_R20.tir.
# The tire's effective rolling radius is NOT explicitly given, so this is only a nominal
# geometric estimate from the 20.5 in diameter designation, not a validated rolling radius.
ARTTU_TIRE_NOMINAL_DIAMETER_M = 20.5 * 0.0254
ARTTU_TIRE_NOMINAL_RADIUS_M = ARTTU_TIRE_NOMINAL_DIAMETER_M / 2.0

# Source motor speed/torque map from Powertrain.xlsx -> Motor!TorqueSpd / TrqSpd.
ARTTU_MOTOR_SPEED_RPM = (
    0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100,
    1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100,
    2200, 2300, 2400, 2500, 2600, 2700, 2800, 2900, 3000, 3100,
    3200, 3300, 3400, 3500, 3600, 3700, 3800, 3900, 4000, 4100,
    4200, 4300, 4400, 4500, 4600,
)
ARTTU_MOTOR_TORQUE_NM = (
    (51.0306904761905,) * 37
    + (44.8436898332690, 36.8215688218961, 30.0627288241220, 25.3370743140982,
       21.0374123095361, 17.0840324267782, 13.4184887499186, 9.99592684833052,
       6.77696534591492, 3.73957484604455)
)


@dataclass(frozen=True)
class ARTTUParameters:
    """Centralized ARTTU inputs for Phase 2B.

    Unknown quantities remain explicit rather than being silently replaced with guesses.
    """

    mass_kg: float = ARTTU_MASS_KG
    wheel_radius_m: float | None = ARTTU_TIRE_NOMINAL_RADIUS_M
    gear_ratio: float = ARTTU_GEAR_RATIO
    drivetrain_efficiency: float | None = None
    rolling_resistance_coeff: float | None = None
    air_density_kg_m3: float = ARTTU_AIR_DENSITY_KG_M3
    drag_coeff: float = ARTTU_CD_MAGNITUDE
    frontal_area_m2: float = ARTTU_FRONTAL_AREA_M2
    road_grade_rad: float = 0.0

    converter_efficiency: float = ARTTU_CONVERTER_EFFICIENCY
    inverter_continuous_power_w: float = ARTTU_INVERTER_CONTINUOUS_POWER_W
    total_battery_power_limit_w: float = ARTTU_TOTAL_BATTERY_POWER_LIMIT_W

    motor_speed_rpm: tuple[float, ...] = ARTTU_MOTOR_SPEED_RPM
    motor_torque_nm: tuple[float, ...] = ARTTU_MOTOR_TORQUE_NM

    @property
    def motor_map_max_rpm(self) -> float:
        return self.motor_speed_rpm[-1]

    def to_vehicle_parameters(self) -> VehicleParameters:
        missing = []
        if self.wheel_radius_m is None:
            missing.append("wheel_radius_m")
        if self.drivetrain_efficiency is None:
            missing.append("drivetrain_efficiency")
        if self.rolling_resistance_coeff is None:
            missing.append("rolling_resistance_coeff")
        if missing:
            raise ValueError(
                "Unknown ARTTU parameter(s): " + ", ".join(missing)
            )
        return VehicleParameters(
            mass_kg=self.mass_kg,
            wheel_radius_m=self.wheel_radius_m,
            gear_ratio=self.gear_ratio,
            drivetrain_efficiency=self.drivetrain_efficiency,
            rolling_resistance_coeff=self.rolling_resistance_coeff,
            air_density_kg_m3=self.air_density_kg_m3,
            drag_coeff=self.drag_coeff,
            frontal_area_m2=self.frontal_area_m2,
            road_grade_rad=self.road_grade_rad,
        )


# Deliberately conservative working configuration for simulation/tests.
# Only the unknown parameters are assigned temporary values here.
ARTTU_WORKING_PARAMETERS = ARTTUParameters(
    wheel_radius_m=ARTTU_TIRE_NOMINAL_RADIUS_M,
    drivetrain_efficiency=0.90,
    rolling_resistance_coeff=0.015,
)
