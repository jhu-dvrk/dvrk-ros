% call methods to make sure they exist and don't trigger syntax errors
function test_arm_move(arm_name)
    addpath('..');
    r = arm(arm_name);
    
    disp('---- Homing');
    r.home();
    
    %%%% joint position move
    disp('---- Joint move');
    % move to 0 position
    p = r.get_state_joint_desired();
    joints_home = p;
    joints_home(:) = 0.0;
    r.move_joint(joints_home);
    % wiggle first two joints, matlab index starts at 1
    amplitude = deg2rad(5.0);
    % first move
    goal = joints_home;
    goal(1:2) = amplitude;
    r.move_joint(goal);
    % second move
    goal = joints_home;
    goal(1:2) = -amplitude;
    r.move_joint(goal);
    % back home
    r.move_joint(joints_home);
    
    %%%% cartesian position move
    disp('---- Cartesian move');
    % make sure tool tip is past RCM on ECM and PSMs
    n = r.robot_name;
    if (strcmp(n, 'ECM') || strncmp(n, 'PSM', 3))
        goal = joints_home;
        goal(3) = 0.12;
        r.move_joint(goal);
    end
    cartesian_home = r.get_position_desired();
    amplitude = 0.03; % 3 cm
    % first move
    goal = cartesian_home;
    goal(1:2, 4) = goal(1:2, 4) + amplitude;
    r.move(goal);
    % second move
    goal = cartesian_home;
    goal(1:2, 4) = goal(1:2, 4) - amplitude;
    r.move(goal);
    % first move
    r.move(cartesian_home);

end
