function tests = test_vehicle_longitudinal_rhs_artu
p = phase_2b_arttu_parameters();
tests = functiontests(localfunctions);

    function testAtZeroSpeed(testCase)
        [a, forces] = vehicle_longitudinal_rhs_artu(0, 51.0306904761905, 51.0306904761905, p);
        verifyGreaterThan(testCase, forces.tractiveTotal_N, 0);
        verifyGreaterThan(testCase, a, 0);
    end

    function testLeftRightAdd(testCase)
        p2 = p;
        p2.rolling_resistance_coeff = 0;
        p2.air_density_kg_m3 = 0;
        p2.drag_coeff = 0;
        p2.frontal_area_m2 = 0;
        [a, forces] = vehicle_longitudinal_rhs_artu(0, 20, 30, p2);
        verifyEqual(testCase, forces.tractiveTotal_N, ...
            forces.tractiveLeft_N + forces.tractiveRight_N, 'AbsTol', 1e-12);
        verifyEqual(testCase, a, forces.netForce_N/p2.mass_kg, 'AbsTol', 1e-12);
    end

    function testTorqueMap(testCase)
        p2 = p;
        p2.rolling_resistance_coeff = 0;
        p2.air_density_kg_m3 = 0;
        p2.drag_coeff = 0;
        p2.frontal_area_m2 = 0;
        rpm = 2500;
        v = rpm*(2*pi/60)*p2.wheel_radius_m/p2.gear_ratio;
        [~, forces] = vehicle_longitudinal_rhs_artu(v, 100, 100, p2);
        verifyEqual(testCase, forces.availableTorque_Nm, 51.0306904761905, 'AbsTol', 1e-12);
    end
end
