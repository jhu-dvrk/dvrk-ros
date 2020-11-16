% call methods to make sure they exist and don't trigger syntax errors
function test_mtm_api(arm_name)
    try
        rosnode list;
    catch
        rosinit;
    end
    r = dvrk.mtm(arm_name)
    disp('---- Enabling (waiting up to 30s)');
    if ~r.enable(30.0)
        error('Unable to enable arm');
    end
    disp('---- Homing (waiting up to 30s)');
    if ~r.home(30.0)
        error('Unable to home arm');
    end

    disp('---- Testing get_state_gripper_current');
    [p, v, e, t] = r.gripper.measured_js()

    disp('-> arm will go limp, hold it and press enter');
    pause;
    r.body.servo_cf([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);

    disp('-> keep holding arm, press any key, gravity compensation will be activated');
    pause;
    r.use_gravity_compensation(true);

    disp('-> keep holding arm, press any key, a force in body frame will be applied (wrist based direction)');
    pause;
    r.body_set_cf_orientation_absolute(false);
    r.body.servo_cf([0.0, 0.0, -2.0, 0.0, 0.0, 0.0]);

    disp('-> keep holding arm, press any key, a force in world frame will be applied (fixed direction)');
    pause;
    r.body_set_cf_orientation_absolute(true);
    r.body.servo_cf([0.0, 0.0, 2.0, 0.0, 0.0, 0.0]);

    disp('-> keep holding arm, press any key, orientation will be locked');
    pause;
    r.lock_orientation_as_is();

    disp('-> keep holding arm, press any key, force will be removed');
    pause;
    r.body.servo_cf([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);

    disp('-> keep holding arm, press any key, orientation will be unlocked');
    pause;
    r.unlock_orientation();

    disp('-> keep holding arm, press any key, arm will freeze in position');
    pause;
    r.move_jp(r.measured_js());

    % don't forget to cleanup
    disp('---- Delete arm class');
    delete(r);
end
