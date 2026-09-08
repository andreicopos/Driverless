# ARTTU Phase 2B — Parameter Inventory

## Known project data

| Parameter | Symbol | Value | Unit | Classification | Source |
|---|---:|---:|---:|---|---|
| Vehicle mass | m | 239 | kg | Known | User-provided |
| Final drive ratio | i_g | 5 | – | Known from workbook | `sm_car_ARTTU_Powertrain.xlsx`, `Driveline` |
| Inverter continuous power | P_inv,max | 43 | kW/inverter | Known | User-provided + Plettenberg MST400-133 manual p.4 |
| Total traction power restriction | P_batt,max | 40 | kW combined | Known project restriction | User-provided |
| Air density | rho | 1.225 | kg/m³ | Known model input | `sm_car_ARTTU_Chassis.xlsx`, `Aero` |
| Frontal reference area | A | 1.2 | m² | Known model input | `sm_car_ARTTU_Chassis.xlsx`, `Aero` |
| Drag coefficient | C_D | 1.8 magnitude | – | Source value; sign convention converted | `sm_car_ARTTU_Chassis.xlsx`, `Aero` stores -1.8 |
| Tire nominal diameter | D | 20.5 | in | Nominal from tire-file name | `sm_car_ARTTU_Chassis.xlsx`, `Tire` |
| Motor torque-speed map | T(ω) | 0–4600 rpm table | rpm / N·m | Known source data | `sm_car_ARTTU_Powertrain.xlsx`, `Motor` / `TrqSpd` |
| Converter efficiency | η_conv | 90 | % | Model input from workbook | `sm_car_ARTTU_Powertrain.xlsx`, `Converter` |

## Unknown ARTTU parameters

- Effective loaded wheel radius.
- Drivetrain mechanical efficiency.
- Rolling resistance coefficient.
- Exact Plettenberg motor model/variant.
- Exact motor continuous/peak torque rating independent of the supplied torque-speed map.
- Exact relationship between the 40 kW battery restriction and inverter/motor mechanical output power.
- Exact motor/inverter thermal derating map.

## Important source discrepancies / interpretation notes

1. The Plettenberg MST400-133 manual states 43 kW maximum continuous power and 58 kW maximum short-term power per controller (p.4). The project restriction supplied by the user is 40 kW combined, so the Phase 2B working model applies the 40 kW combined cap before considering per-inverter headroom.
2. The Powertrain workbook's `Converter` sheet lists 26 kW and 130 A, which does not match the 43 kW controller rating supplied by the manual/user. The present model does not use 26 kW as the hard limit; this discrepancy should be resolved with the team's actual inverter configuration.
3. The workbook stores aerodynamic `CD=-1.8`. The longitudinal drag equation uses positive drag magnitude and subtracts the resulting force from the force balance, hence `C_D=1.8` is used in the longitudinal model.
4. The `Tire` sheet references `Hoosier_20p5x7_13_R20.tir`, but does not give a validated loaded rolling radius. The current working model uses half of the 20.5-inch nominal diameter only as a temporary simulation value.
5. The torque-speed map ends at 4600 rpm. The implementation does not extrapolate beyond this supplied range.
