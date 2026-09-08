import pytest

from vehicle_model import ARTTU_WORKING_PARAMETERS, command_from_accelerator, motor_rpm_from_vehicle_speed
from vehicle_model.arttu_parameters import ARTTU_INVERTER_CONTINUOUS_POWER_W, ARTTU_TOTAL_BATTERY_POWER_LIMIT_W


def test_mass_and_gearbox_inputs_are_artu_values():
    assert ARTTU_WORKING_PARAMETERS.mass_kg == pytest.approx(239.0)
    assert ARTTU_WORKING_PARAMETERS.gear_ratio == pytest.approx(5.0)


def test_motor_map_flat_torque_region():
    out = command_from_accelerator(0.0, 1.0, ARTTU_WORKING_PARAMETERS)
    assert out.torque_available_left_nm == pytest.approx(51.0306904761905)
    assert out.torque_available_right_nm == pytest.approx(51.0306904761905)


def test_motor_speed_from_vehicle_speed():
    rpm = motor_rpm_from_vehicle_speed(10.0, 5.0, ARTTU_WORKING_PARAMETERS.wheel_radius_m)
    assert rpm > 0


def test_combined_battery_power_limit_is_not_above_user_limit():
    out = command_from_accelerator(20.0, 1.0, ARTTU_WORKING_PARAMETERS)
    assert out.battery_power_total_w <= ARTTU_TOTAL_BATTERY_POWER_LIMIT_W + 1e-9


def test_inverter_rating_is_higher_than_total_cap():
    assert ARTTU_INVERTER_CONTINUOUS_POWER_W * 2 > ARTTU_TOTAL_BATTERY_POWER_LIMIT_W


def test_power_limit_keeps_equal_motor_split():
    out = command_from_accelerator(20.0, 1.0, ARTTU_WORKING_PARAMETERS)
    assert out.torque_command_left_nm == pytest.approx(out.torque_command_right_nm)
    assert out.battery_power_left_w == pytest.approx(out.battery_power_right_w)


def test_battery_limit_eventually_becomes_active():
    # At sufficiently high motor speed, the low-speed torque plateau would exceed the
    # 36 kW mechanical equivalent of the 40 kW battery limit.
    limited = command_from_accelerator(19.0, 1.0, ARTTU_WORKING_PARAMETERS)
    assert limited.total_battery_power_limited
    assert limited.battery_power_total_w <= ARTTU_TOTAL_BATTERY_POWER_LIMIT_W + 1e-9
