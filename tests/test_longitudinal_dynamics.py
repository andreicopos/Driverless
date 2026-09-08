import math

import pytest

from vehicle_model import (
    DEFAULT_TEST_PARAMETERS,
    MotorCommand,
    MotorLimits,
    LongitudinalState,
    aerodynamic_drag_force,
    compute_forces,
    model_step,
)


def test_constant_force_acceleration_without_resistance():
    p = DEFAULT_TEST_PARAMETERS.__class__(
        mass_kg=250.0,
        wheel_radius_m=0.25,
        gear_ratio=1.0,
        drivetrain_efficiency=1.0,
        rolling_resistance_coeff=0.0,
        air_density_kg_m3=0.0,
        drag_coeff=0.0,
        frontal_area_m2=0.0,
    )
    command = MotorCommand(250.0, 0.0)
    forces = compute_forces(0.0, command, p)
    assert forces.tractive_total_n == pytest.approx(1000.0)

    state, output = model_step(LongitudinalState(0.0, 0.0), command, 0.1, p)
    assert output.acceleration_mps2 == pytest.approx(4.0)
    assert state.velocity_mps == pytest.approx(0.4)


def test_left_right_forces_add():
    p = DEFAULT_TEST_PARAMETERS.__class__(
        mass_kg=250.0,
        wheel_radius_m=0.25,
        gear_ratio=1.0,
        drivetrain_efficiency=1.0,
        rolling_resistance_coeff=0.0,
        air_density_kg_m3=0.0,
        drag_coeff=0.0,
        frontal_area_m2=0.0,
    )
    forces = compute_forces(0.0, MotorCommand(100.0, 100.0), p)
    assert forces.tractive_left_n == pytest.approx(400.0)
    assert forces.tractive_right_n == pytest.approx(400.0)
    assert forces.tractive_total_n == pytest.approx(800.0)


def test_torque_limit_saturates_each_motor():
    p = DEFAULT_TEST_PARAMETERS.__class__(
        mass_kg=250.0,
        wheel_radius_m=0.25,
        gear_ratio=1.0,
        drivetrain_efficiency=1.0,
        rolling_resistance_coeff=0.0,
        air_density_kg_m3=0.0,
        drag_coeff=0.0,
        frontal_area_m2=0.0,
    )
    forces = compute_forces(
        0.0,
        MotorCommand(150.0, -150.0),
        p,
        MotorLimits(max_torque_nm=100.0),
    )
    assert forces.tractive_left_n == pytest.approx(400.0)
    assert forces.tractive_right_n == pytest.approx(-400.0)
    assert forces.net_force_n == pytest.approx(0.0)


def test_aero_drag_is_quadratic():
    v1 = aerodynamic_drag_force(10.0, DEFAULT_TEST_PARAMETERS)
    v2 = aerodynamic_drag_force(20.0, DEFAULT_TEST_PARAMETERS)
    assert v2 / v1 == pytest.approx(4.0)


def test_no_command_on_flat_road_decelerates_from_resistance():
    state, output = model_step(
        LongitudinalState(0.0, 20.0),
        MotorCommand(0.0, 0.0),
        0.1,
        DEFAULT_TEST_PARAMETERS,
    )
    assert output.acceleration_mps2 < 0.0
    assert state.velocity_mps < 20.0


def test_forward_euler_velocity_matches_equation_for_small_step():
    p = DEFAULT_TEST_PARAMETERS.__class__(
        mass_kg=250.0,
        wheel_radius_m=0.25,
        gear_ratio=1.0,
        drivetrain_efficiency=1.0,
        rolling_resistance_coeff=0.0,
        air_density_kg_m3=0.0,
        drag_coeff=0.0,
        frontal_area_m2=0.0,
    )
    state, output = model_step(
        LongitudinalState(0.0, 5.0),
        MotorCommand(50.0, 50.0),
        0.01,
        p,
    )
    expected_a = 400.0 / 250.0
    assert output.acceleration_mps2 == pytest.approx(expected_a)
    assert state.velocity_mps == pytest.approx(5.0 + expected_a * 0.01)


def test_parameter_validation():
    with pytest.raises(ValueError):
        DEFAULT_TEST_PARAMETERS.__class__(
            mass_kg=0.0,
            wheel_radius_m=0.23,
            gear_ratio=4.5,
            drivetrain_efficiency=0.95,
            rolling_resistance_coeff=0.015,
            air_density_kg_m3=1.225,
            drag_coeff=0.9,
            frontal_area_m2=1.2,
        ).validate()
