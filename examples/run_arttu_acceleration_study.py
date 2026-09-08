"""ARTTU Phase 2B: full-throttle 0-80 km/h acceleration study.

This is an engineering study, not a validation against measured acceleration yet.
Only the ~80 km/h terminal-speed measurement is currently used as a real-world benchmark.
"""

import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt

from vehicle_model import (
    ARTTU_WORKING_PARAMETERS,
    LongitudinalState,
    MotorCommand,
    command_from_accelerator,
    model_step,
    rolling_resistance_force,
    aerodynamic_drag_force,
    grade_force,
)

TARGET_SPEED_MPS = 80.0 / 3.6
DT_S = 0.005
MAX_TIME_S = 30.0


def main() -> None:
    p = ARTTU_WORKING_PARAMETERS
    vp = p.to_vehicle_parameters()

    state = LongitudinalState(position_m=0.0, velocity_mps=0.0)
    records: list[dict[str, float]] = []
    reached_80 = None

    steps = int(MAX_TIME_S / DT_S)
    for k in range(steps + 1):
        t = k * DT_S
        drive = command_from_accelerator(state.velocity_mps, 1.0, p)

        if k == steps:
            acceleration = 0.0
            forces = None
        else:
            next_state, output = model_step(
                state,
                MotorCommand(
                    drive.torque_command_left_nm,
                    drive.torque_command_right_nm,
                ),
                DT_S,
                vp,
            )
            acceleration = output.acceleration_mps2
            forces = output.forces

        records.append({
            "time_s": t,
            "position_m": state.position_m,
            "velocity_mps": state.velocity_mps,
            "velocity_kmh": state.velocity_mps * 3.6,
            "acceleration_mps2": acceleration,
            "motor_rpm": drive.motor_rpm_left,
            "torque_left_nm": drive.torque_command_left_nm,
            "torque_right_nm": drive.torque_command_right_nm,
            "tractive_force_n": forces.tractive_total_n if forces else 0.0,
            "rolling_resistance_n": forces.rolling_resistance_n if forces else rolling_resistance_force(vp),
            "aero_drag_n": forces.aerodynamic_drag_n if forces else aerodynamic_drag_force(state.velocity_mps, vp),
            "grade_force_n": forces.grade_force_n if forces else grade_force(vp),
            "net_force_n": forces.net_force_n if forces else 0.0,
            "battery_power_kw": drive.battery_power_total_w / 1000.0,
            "mechanical_power_kw": drive.mechanical_power_total_w / 1000.0,
            "power_limited": float(drive.power_limited),
        })

        if reached_80 is None and state.velocity_mps >= TARGET_SPEED_MPS:
            reached_80 = t
            break

        if k < steps:
            state = next_state

    out_dir = Path(__file__).resolve().parent.parent / "analysis"
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "arttu_0_80_full_throttle.csv"

    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=records[0].keys())
        writer.writeheader()
        writer.writerows(records)

    t = [r["time_s"] for r in records]
    v = [r["velocity_kmh"] for r in records]
    a = [r["acceleration_mps2"] for r in records]
    pwr = [r["battery_power_kw"] for r in records]
    rpm = [r["motor_rpm"] for r in records]
    fx = [r["tractive_force_n"] for r in records]
    frr = [r["rolling_resistance_n"] for r in records]
    faero = [r["aero_drag_n"] for r in records]
    fnet = [r["net_force_n"] for r in records]

    fig, ax = plt.subplots()
    ax.plot(t, v)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Vehicle speed [km/h]")
    ax.set_title("ARTTU 0–80 km/h Full-Throttle Simulation")
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(out_dir / "01_speed.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots()
    ax.plot(t, a)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Acceleration [m/s²]")
    ax.set_title("Longitudinal Acceleration")
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(out_dir / "02_acceleration.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots()
    ax.plot(t, pwr)
    ax.axhline(p.total_battery_power_limit_w / 1000.0)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Battery power [kW]")
    ax.set_title("Combined Battery Power")
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(out_dir / "03_battery_power.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots()
    ax.plot(t, rpm)
    ax.axhline(p.motor_map_max_rpm)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Motor speed [rpm]")
    ax.set_title("Motor Speed")
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(out_dir / "04_motor_rpm.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots()
    ax.plot(t, fx, label="Traction")
    ax.plot(t, frr, label="Rolling resistance")
    ax.plot(t, faero, label="Aerodynamic drag")
    ax.plot(t, fnet, label="Net force")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Force [N]")
    ax.set_title("Longitudinal Force Breakdown")
    ax.legend()
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(out_dir / "05_force_breakdown.png", dpi=150)
    plt.close(fig)

    last = records[-1]
    print("ARTTU Phase 2B — 0–80 km/h full-throttle study")
    print(f"dt = {DT_S:.3f} s")
    print(f"mass = {p.mass_kg:.1f} kg")
    print(f"gear ratio = {p.gear_ratio:.2f}")
    print(f"working wheel radius = {p.wheel_radius_m:.5f} m (temporary)")
    print(f"combined battery limit = {p.total_battery_power_limit_w / 1000:.1f} kW")
    print(f"per-inverter continuous rating = {p.inverter_continuous_power_w / 1000:.1f} kW")
    print(f"converter efficiency = {p.converter_efficiency:.2f}")
    if reached_80 is None:
        print("0–80 km/h time: NOT REACHED")
    else:
        print(f"0–80 km/h time: {reached_80:.3f} s")
    print(f"final simulated speed: {last['velocity_kmh']:.2f} km/h")
    print(f"final motor rpm: {last['motor_rpm']:.0f} rpm")
    print(f"final battery power: {last['battery_power_kw']:.2f} kW")
    print(f"results: {out_dir}")


if __name__ == "__main__":
    main()
