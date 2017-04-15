% call methods to make sure they exist and don't trigger syntax errors
function test_mtm_api(arm_name)
    % test base class api
    test_arm_api(arm_name)

    % PSM specific api
    r = mtm(arm_name)
    disp('---- Testing get_state_gripper_current')
    [p, v, e, t] = r.get_state_gripper_current()

end
