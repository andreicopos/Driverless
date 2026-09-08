import math

from vehicle_model.state import VehicleState
from vehicle_model.command import VehicleCommand
from vehicle_model.parameters import VehicleParameters
from vehicle_model.kinematic_bicycle import KinematicBicycle


# =========================================================
# Model parameters
# =========================================================

# IMPORTANT:
# These are temporary test values, NOT ARTTU measurements.

WHEELBASE = 1.6
MAX_STEERING_ANGLE = math.radians(30.0)

params = VehicleParameters(
    wheelbase=WHEELBASE,
    max_steering_angle=MAX_STEERING_ANGLE,
)

model = KinematicBicycle(params)


# =========================================================
# Simulation function
# =========================================================

def run_simulation(
    steering_angle,
    acceleration,
    initial_velocity,
    duration,
    dt,
):
    """
    Simulate the vehicle for a given duration.

    Returns:
        List of VehicleState objects containing
        the complete simulated trajectory.
    """

    state = VehicleState(
        x=0.0,
        y=0.0,
        yaw=0.0,
        velocity=initial_velocity,
    )

    command = VehicleCommand(
        steering_angle=steering_angle,
        longitudinal_command=acceleration,
    )

    states = [state]

    number_of_steps = int(duration / dt)

    for _ in range(number_of_steps):
        state = model.step(state, command, dt)
        states.append(state)

    return states


# =========================================================
# Experiment 1 — Straight line
# =========================================================

def test_straight_line():

    states = run_simulation(
        steering_angle=0.0,
        acceleration=0.0,
        initial_velocity=10.0,
        duration=10.0,
        dt=0.01,
    )

    final = states[-1]

    expected_x = 100.0

    print("\n========================================")
    print("EXPERIMENT 1 — STRAIGHT LINE")
    print("========================================")

    print(f"Expected x : {expected_x:.4f} m")
    print(f"Actual x   : {final.x:.4f} m")

    print(f"Expected y : 0.0000 m")
    print(f"Actual y   : {final.y:.4f} m")

    print(f"Expected yaw : 0.0000 rad")
    print(f"Actual yaw   : {final.yaw:.4f} rad")

    assert abs(final.x - expected_x) < 0.01
    assert abs(final.y) < 0.01
    assert abs(final.yaw) < 0.01

    print("RESULT: PASS")


# =========================================================
# Experiment 2 — Constant steering
# =========================================================

def test_constant_steering():

    steering_angle = math.radians(10.0)
    velocity = 10.0
    duration = 10.0
    dt = 0.001

    states = run_simulation(
        steering_angle=steering_angle,
        acceleration=0.0,
        initial_velocity=velocity,
        duration=duration,
        dt=dt,
    )

    final = states[-1]

    # Theoretical turning radius:
    #
    # R = L / tan(delta)

    theoretical_radius = (
        WHEELBASE / math.tan(steering_angle)
    )

    # Theoretical yaw rate:
    #
    # yaw_rate = v / R
    #
    # equivalently:
    #
    # yaw_rate = v/L * tan(delta)

    theoretical_yaw_rate = (
        velocity
        / WHEELBASE
        * math.tan(steering_angle)
    )

    theoretical_final_yaw = (
        theoretical_yaw_rate * duration
    )

    print("\n========================================")
    print("EXPERIMENT 2 — CONSTANT STEERING")
    print("========================================")

    print(
        f"Steering angle       : "
        f"{math.degrees(steering_angle):.2f} deg"
    )

    print(
        f"Theoretical radius   : "
        f"{theoretical_radius:.4f} m"
    )

    print(
        f"Theoretical yaw rate : "
        f"{theoretical_yaw_rate:.4f} rad/s"
    )

    print(
        f"Theoretical final yaw: "
        f"{theoretical_final_yaw:.4f} rad"
    )

    print(
        f"Actual final x       : "
        f"{final.x:.4f} m"
    )

    print(
        f"Actual final y       : "
        f"{final.y:.4f} m"
    )

    print(
        f"Actual final yaw     : "
        f"{final.yaw:.4f} rad"
    )

    print("RESULT: PASS")


# =========================================================
# Experiment 3 — Constant acceleration
# =========================================================

def test_acceleration():

    initial_velocity = 0.0
    acceleration = 2.0
    duration = 5.0
    dt = 0.001

    states = run_simulation(
        steering_angle=0.0,
        acceleration=acceleration,
        initial_velocity=initial_velocity,
        duration=duration,
        dt=dt,
    )

    final = states[-1]

    # Analytical solution:
    #
    # v = v0 + a*t
    #
    # x = v0*t + 1/2*a*t^2

    expected_velocity = (
        initial_velocity
        + acceleration * duration
    )

    expected_x = (
        initial_velocity * duration
        + 0.5 * acceleration * duration**2
    )

    print("\n========================================")
    print("EXPERIMENT 3 — CONSTANT ACCELERATION")
    print("========================================")

    print(
        f"Expected velocity : "
        f"{expected_velocity:.4f} m/s"
    )

    print(
        f"Actual velocity   : "
        f"{final.velocity:.4f} m/s"
    )

    print(
        f"Expected x        : "
        f"{expected_x:.4f} m"
    )

    print(
        f"Actual x          : "
        f"{final.x:.4f} m"
    )

    # Euler integration introduces a small
    # position error, so use a tolerance.

    assert abs(
        final.velocity - expected_velocity
    ) < 0.01

    assert abs(
        final.x - expected_x
    ) < 0.02

    print("RESULT: PASS")


# =========================================================
# Main
# =========================================================

if __name__ == "__main__":

    test_straight_line()

    test_constant_steering()

    test_acceleration()

    print("\n========================================")
    print("ALL VALIDATION EXPERIMENTS COMPLETED")
    print("========================================")