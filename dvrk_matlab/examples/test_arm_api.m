% call methods to make sure they exist and don't trigger syntax errors
function test_arm_api(arm_name)
    try
        rosnode list;
    catch
        rosinit;
    end
    r = dvrk.arm(arm_name);
    disp('---- Enabling (waiting up to 30s)');
    if ~r.enable(30.0)
        error('Unable to enable arm');
    end
    disp('---- Homing (waiting up to 30s)');
    if ~r.home(30.0)
        error('Unable to home arm');
    end
    disp('---- Setpoint cartesian');
    [p, t] = r.setpoint_cp()
    disp('---- Setpoint joint');
    [p, v, e, t] = r.setpoint_js()
    disp('---- Measured cartesian');
    [p, t] = r.measured_cp()
    disp('---- Measured joint');
    [p, v, e, t] = r.measured_js()
    disp('---- Measured twist')
    [v, t] = r.measured_cv()
    disp('---- Measured body wrench')
    [e, t] = r.body.measured_cf()
    disp('---- Measured spatial wrench')
    [e, t] = r.spatial.measured_cf()

    % don't forget to cleanup
    disp('---- Delete arm class');
    delete(r);
end
