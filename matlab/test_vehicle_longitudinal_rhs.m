function tests = test_vehicle_longitudinal_rhs
p = phase_2b_test_parameters();
tests = functiontests(localfunctions);

    function testZeroTorqueResists(testCase)
        [a, forces] = vehicle_longitudinal_rhs(20, 0, 0, p);
        verifyLessThan(testCase, a, 0);
        verifyGreaterThan(testCase, forces.rollingResistance_N, 0);
        verifyGreaterThan(testCase, forces.aeroDrag_N, 0);
    end

    function testLeftRightSum(testCase)
        p2 = p;
        p2.rolling_resistance_coeff = 0;
        p2.air_density_kg_m3 = 0;
        p2.drag_coeff = 0;
        p2.frontal_area_m2 = 0;
        [a, forces] = vehicle_longitudinal_rhs(0, 100, 100, p2);
        verifyEqual(testCase, forces.tractiveTotal_N, ...
            forces.tractiveLeft_N + forces.tractiveRight_N, 'AbsTol', 1e-12);
        verifyEqual(testCase, a, forces.netForce_N / p2.mass_kg, 'AbsTol', 1e-12);
    end
end
