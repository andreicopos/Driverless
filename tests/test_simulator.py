import math

from vehicle_model.state import VehicleState
from vehicle_model.command import VehicleCommand
from vehicle_model.parameters import VehicleParameters
from vehicle_model.kinematic_bicycle import KinematicBicycle


def test_straight_line():
    params = VehicleParameters(
        wheelbase=1.6,
        max_steering_angle=math.radians(30),
    )

    model = KinematicBicycle(params)

    state = VehicleState(
        x=0,
        y=0,
        yaw=0,
        velocity=10,
    )

    command = VehicleCommand(
        steering_angle=0,
        longitudinal_command=0,
    )

    new_state = model.step(state, command, 0.1)

    assert math.isclose(new_state.x, 1.0)
    assert math.isclose(new_state.y, 0.0)
    assert math.isclose(new_state.yaw, 0.0)


def test_velocity_change():
    params = VehicleParameters(
        wheelbase=1.6,
        max_steering_angle=math.radians(30),
    )

    model = KinematicBicycle(params)

    state = VehicleState(
        velocity=0.0,
    )

    command = VehicleCommand(
        longitudinal_command=2.0,
    )

    new_state = model.step(state, command, 1.0)

    assert math.isclose(new_state.velocity, 2.0)


def test_steering_saturation():
    params = VehicleParameters(
        wheelbase=1.6,
        max_steering_angle=math.radians(30),
    )

    model = KinematicBicycle(params)

    state = VehicleState(
        velocity=10.0,
    )

    command = VehicleCommand(
        steering_angle=math.radians(60),
    )

    new_state = model.step(state, command, 0.01)

    expected_yaw_rate = (
        10.0 / 1.6 * math.tan(math.radians(30))
    )

    assert math.isclose(
        new_state.yaw,
        expected_yaw_rate * 0.01,
    )