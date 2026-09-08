from vehicle_model import ARTTU_WORKING_PARAMETERS, LongitudinalState, MotorCommand, command_from_accelerator, model_step


def main() -> None:
    p = ARTTU_WORKING_PARAMETERS
    vp = p.to_vehicle_parameters()
    state = LongitudinalState(position_m=0.0, velocity_mps=0.0)
    dt = 0.01

    print("ARTTU Phase 2B longitudinal demo")
    print("mass =", p.mass_kg, "kg")
    print("gear ratio =", p.gear_ratio)
    print("nominal wheel radius =", p.wheel_radius_m, "m (temporary; loaded radius still unknown)")

    for _ in range(2000):
        drive = command_from_accelerator(state.velocity_mps, 1.0, p)
        state, out = model_step(
            state,
            MotorCommand(drive.torque_command_left_nm, drive.torque_command_right_nm),
            dt,
            vp,
        )

    print(f"after 20 s: v = {state.velocity_mps:.2f} m/s ({state.velocity_mps*3.6:.1f} km/h)")
    print(f"           x = {state.position_m:.1f} m")
    print(f"           a = {out.acceleration_mps2:.2f} m/s^2")
    print(f"           motor rpm = {drive.motor_rpm_left:.0f}")
    print(f"           motor torque = {drive.torque_command_left_nm:.2f} Nm")
    print(f"           battery power = {drive.battery_power_w/1000:.2f} kW")


if __name__ == "__main__":
    main()
