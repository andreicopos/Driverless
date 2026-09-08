function [a, forces] = vehicle_longitudinal_rhs_artu(v, torqueLeft, torqueRight, p)
% ARTTU Phase 2B longitudinal RHS using the supplied torque-speed map and
% combined battery power limit. Positive torque = propulsion.

rpm = v / p.wheel_radius_m * p.gear_ratio * 60/(2*pi);

if rpm < p.motor_speed_rpm(1) || rpm > p.motor_speed_rpm(end)
    Tavailable = 0;
else
    Tavailable = interp1(p.motor_speed_rpm, p.motor_torque_nm, rpm, 'linear');
end

% Equal-split first implementation: normalized torque request is not yet
% a final accelerator/controller model. The caller supplies requested torques.
Tleft = max(-Tavailable, min(Tavailable, torqueLeft));
Tright = max(-Tavailable, min(Tavailable, torqueRight));

omega = rpm*2*pi/60;
PmechReq = max(0,Tleft*omega) + max(0,Tright*omega);
PmechLimit = p.total_battery_power_limit_w * p.converter_efficiency;
if PmechReq > PmechLimit && PmechReq > 0
    scale = PmechLimit/PmechReq;
    Tleft = Tleft*scale;
    Tright = Tright*scale;
end

FxLeft = Tleft*p.gear_ratio*p.drivetrain_efficiency/p.wheel_radius_m;
FxRight = Tright*p.gear_ratio*p.drivetrain_efficiency/p.wheel_radius_m;
FxTraction = FxLeft + FxRight;

Frr = p.rolling_resistance_coeff*p.mass_kg*9.80665*cos(p.road_grade_rad);
Faero = 0.5*p.air_density_kg_m3*p.drag_coeff*p.frontal_area_m2*v^2;
Fgrade = p.mass_kg*9.80665*sin(p.road_grade_rad);
Fnet = FxTraction - Frr - Faero - Fgrade;
a = Fnet/p.mass_kg;

forces = struct( ...
    'motorRpm', rpm, ...
    'availableTorque_Nm', Tavailable, ...
    'tractiveLeft_N', FxLeft, ...
    'tractiveRight_N', FxRight, ...
    'tractiveTotal_N', FxTraction, ...
    'rollingResistance_N', Frr, ...
    'aeroDrag_N', Faero, ...
    'gradeForce_N', Fgrade, ...
    'netForce_N', Fnet);
end
