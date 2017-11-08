% call methods to make sure they exist and don't trigger syntax errors
function test_arm_api(arm_name)
    addpath('..')
    r = arm(arm_name);
    disp('---- Homing')
    r.home()
    disp('---- Desired cartesian');
    [p, t] = r.get_position_desired()
    [p, t] = r.get_position_local_desired()
    disp('---- Desired joint');
    [p, v, e, t] = r.get_state_joint_desired()
    disp('---- Current cartesian');
    [p, t] = r.get_position_current()
    [p, t] = r.get_position_local_current()
    disp('---- Current joint');
    [p, v, e, t] = r.get_state_joint_current()
    disp('---- Current twist')
    [v, t] = r.get_twist_body_current()
    disp('---- Current wrench')
    [e, t] = r.get_wrench_current()
end
