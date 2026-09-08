function p = phase_2b_arttu_parameters()
% ARTTU Phase 2B working parameters from supplied project data.
% Unknown quantities are explicitly marked below.

p.mass_kg = 239.0;                     % user-provided
p.gear_ratio = 5.0;                    % Powertrain.xlsx -> Driveline
p.air_density_kg_m3 = 1.225;           % Chassis.xlsx -> Aero
p.frontal_area_m2 = 1.2;               % Chassis.xlsx -> Aero
p.drag_coeff = 1.8;                    % source stores CD=-1.8; use magnitude here
p.road_grade_rad = 0.0;

% Temporary working values: replace when measured/confirmed.
p.wheel_radius_m = 20.5*0.0254/2;      % nominal tire diameter only, not loaded radius
p.drivetrain_efficiency = 0.90;        % working placeholder
p.rolling_resistance_coeff = 0.015;    % working placeholder

% Plettenberg MST400-133 + project restriction
p.inverter_continuous_power_w = 43000.0;
p.total_battery_power_limit_w = 40000.0;
p.converter_efficiency = 0.90;         % Powertrain.xlsx -> Converter

p.motor_speed_rpm = 0:100:4600;
p.motor_torque_nm = [ ...
51.0306904761905*ones(1,37), ...
44.8436898332690, 36.8215688218961, 30.0627288241220, 25.3370743140982, ...
21.0374123095361, 17.0840324267782, 13.4184887499186, 9.99592684833052, ...
6.77696534591492, 3.73957484604455];
end
