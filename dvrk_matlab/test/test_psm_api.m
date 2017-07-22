% call methods to make sure they exist and don't trigger syntax errors
function test_psm_api(arm_name)
    % test base class api
    test_arm_api(arm_name)

    % PSM specific api
    r = psm(arm_name)
    disp('---- Testing get_state_jaw_current');
    [p, v, e, t] = r.get_state_jaw_current()
    disp('---- Testing get_state_jaw_desired');
    [p, v, e, t] = r.get_state_jaw_desired()

end
