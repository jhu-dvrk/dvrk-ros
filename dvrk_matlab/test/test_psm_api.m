% call methods to make sure they exist and don't trigger syntax errors
function test_psm_api(arm_name)

    % PSM specific api
    r = psm(arm_name);
    disp('---- Testing get_state_jaw_current');
    [p, v, e, t] = r.get_state_jaw_current();
    disp('---- Testing get_state_jaw_desired');
    [p, v, e, t] = r.get_state_jaw_desired();

    disp(' -> Press any key to start a test');
    disp(' -> Close and open after trajectory joint move, jaw should close/open twice');
    pause;
    r.move_joint(r.get_state_joint_desired())
    r.insert_tool(0.12)
    r.close_jaw()
    r.open_jaw()
    r.close_jaw()
    r.open_jaw()

end
