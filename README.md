# ARTTU Phase 2B — Longitudinal Vehicle Dynamics

This version replaces the generic test-only drivetrain inputs with the supplied ARTTU powertrain/chassis data wherever the files actually support them.

## Current model

The longitudinal model remains:

`m * dv/dt = Fx_left + Fx_right - Frr - Faero - Fgrade`

with:

`Twheel = Tmotor * gear_ratio * eta_drivetrain`

`Fx = Twheel / Rwheel`

`Frr = Crr * m * g * cos(theta)`

`Faero = 0.5 * rho * Cd * A * v^2`

`Fgrade = m * g * sin(theta)`

## ARTTU-specific inputs now integrated

- Vehicle mass = 239 kg (user-provided)
- Final drive ratio = 5 (Powertrain.xlsx)
- Motor torque-speed map = Powertrain.xlsx `Motor` / `TrqSpd`
- Air density = 1.225 kg/m^3 (Chassis.xlsx)
- Frontal reference area = 1.2 m^2 (Chassis.xlsx)
- Drag coefficient magnitude = 1.8; source workbook stores -1.8 because of its sign convention
- Plettenberg MST400-133 continuous power = 43 kW/controller (manual p.4)
- User project battery restriction = 40 kW combined
- Converter efficiency = 90% from the Powertrain workbook, used as the present working conversion-efficiency input

## Still unknown

- Effective loaded tire radius
- Mechanical drivetrain efficiency
- Rolling resistance coefficient
- Exact motor variant
- Thermal derating map
- Exact interpretation of the 40 kW combined battery restriction relative to electrical/mechanical power

## Power limiting

The first ARTTU command abstraction splits requested torque equally between the two rear motors. If total requested mechanical power exceeds:

`40 kW * 0.90 = 36 kW mechanical`

the two positive torque requests are scaled proportionally. This is a modeling interpretation of the user's 40 kW battery restriction plus the workbook's 90% converter efficiency, and must be confirmed against the team's electrical power definition.

The per-inverter 43 kW continuous rating is retained as an independent hardware ceiling. It is not currently the active bottleneck because the combined 40 kW battery restriction is lower.

## Run tests

From this directory:

```bash
python -m pytest -q
```

## Next step

Resolve the three biggest remaining inputs: effective rolling radius, drivetrain efficiency, and the exact motor model. Then connect the longitudinal drivetrain block to the Phase 2A kinematic bicycle model and build the Simulink implementation around the same equations.

## Added next-step power-limit and acceleration study

The ARTTU drivetrain now applies both:

1. the 43 kW continuous power ceiling for each MST400-133 controller, and
2. the project-level 40 kW combined battery-side power restriction.

For the present working model, the 40 kW battery limit is converted to a 36 kW positive mechanical-power limit using the supplied 90% converter-efficiency value. This interpretation must be confirmed against the team's actual logged electrical-power definition.

The model then preserves the equal left/right torque split while scaling both motor torques when a power limit is reached.

Run the next engineering study with:

```bash
python -m examples.run_arttu_acceleration_study
```

Outputs are written to `analysis/`:

- `arttu_0_80_full_throttle.csv`
- `01_speed.png`
- `02_acceleration.png`
- `03_battery_power.png`
- `04_motor_rpm.png`
- `05_force_breakdown.png`

This is not yet a measured acceleration validation. The current experimentally measured benchmark is the approximately 80 km/h track top speed.
