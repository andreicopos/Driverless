"""Brake parameter record for Phase 2B inventory.

A full brake-torque model is intentionally not enabled yet because the supplied
workbook does not document enough of the caliper/disc force convention to safely
infer the exact Simscape torque equation.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class BrakeAxleParameters:
    mean_disc_radius_m: float = 0.107
    friction_static: float = 0.35
    friction_kinetic: float = 0.35
    viscous_coeff_nm_per_rad_s: float = 0.001
    n_pads: int = 4
    caliper_cylinder_diameter_m: float = 0.019
    max_pressure_bar: float = 60.0
    actuator_cutoff_hz: float = 40.0


ARTTU_FRONT_BRAKE = BrakeAxleParameters()
ARTTU_REAR_BRAKE = BrakeAxleParameters()
