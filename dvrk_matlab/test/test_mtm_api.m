% call methods to make sure they exist and don't trigger syntax errors
function test_mtm_api(arm_name)

    % MTM specific api
    r = mtm(arm_name)
    disp('---- Testing get_state_gripper_current');
    [p, v, e, t] = r.get_state_gripper_current()

    disp('-> arm will go limp, hold it and press enter');
    pause;
    r.set_wrench_body([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);

    disp('-> keep holding arm, press any key, a force in body frame will be applied');
    pause;
    r.set_wrench_body_orientation_absolute(false);
    r.set_wrench_body([0.0, 0.0, -2.0, 0.0, 0.0, 0.0]);

    disp('-> keep holding arm, press any key, a force in world frame will be applied');
    pause;
    r.set_wrench_body_orientation_absolute(true);
    r.set_wrench_body([0.0, 0.0, 2.0, 0.0, 0.0, 0.0]);

    disp('-> keep holding arm, press any key, orientation will be locked');
    pause;
    r.lock_orientation_as_is();

    disp('-> keep holding arm, press any key, force will be removed');
    pause;
    r.set_wrench_body([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);

    disp('-> keep holding arm, press any key, orientation will be unlocked');
    pause;
    r.unlock_orientation();

    disp('-> keep holding arm, press any key, arm will freeze in position');
    pause;
    r.move_joint(r.get_state_joint_desired())

    disp('<- bye');
end
