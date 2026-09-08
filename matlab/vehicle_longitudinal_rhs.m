function [a, forces] = vehicle_longitudinal_rhs(v, torqueLeft, torqueRight, p)
% First Phase 2B longitudinal dynamics implementation.
% Temporary parameter values must be supplied explicitly by the caller.
%
% p.mass_kg
% p.wheel_radius_m
% p.gear_ratio
% p.drivetrain_efficiency
% p.rolling_resistance_coeff
% p.air_density_kg_m3
% p.drag_coeff
% p.frontal_area_m2
% p.road_grade_rad

validateattributes(v, {'numeric'}, {'scalar', 'real', 'finite'});
validateattributes(torqueLeft, {'numeric'}, {'scalar', 'real', 'finite'});
validateattributes(torqueRight, {'numeric'}, {'scalar', 'real', 'finite'});

FxLeft = torqueLeft * p.gear_ratio * p.drivetrain_efficiency / p.wheel_radius_m;
FxRight = torqueRight * p.gear_ratio * p.drivetrain_efficiency / p.wheel_radius_m;
FxTraction = FxLeft + FxRight;

Frr = p.rolling_resistance_coeff * p.mass_kg * 9.80665 * cos(p.road_grade_rad);
Faero = 0.5 * p.air_density_kg_m3 * p.drag_coeff * p.frontal_area_m2 * v^2;
Fgrade = p.mass_kg * 9.80665 * sin(p.road_grade_rad);

Fnet = FxTraction - Frr - Faero - Fgrade;
a = Fnet / p.mass_kg;

forces = struct( ...
    'tractiveLeft_N', FxLeft, ...
    'tractiveRight_N', FxRight, ...
    'tractiveTotal_N', FxTraction, ...
    'rollingResistance_N', Frr, ...
    'aeroDrag_N', Faero, ...
    'gradeForce_N', Fgrade, ...
    'netForce_N', Fnet);
end
