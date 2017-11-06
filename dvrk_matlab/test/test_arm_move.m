% call methods to make sure they exist and don't trigger syntax errors
% this test program will make the arm move!
function test_arm_move(arm_name)
    addpath('..');
    r = arm(arm_name);

    disp('---- Homing');
    r.home();

    %%%% for direct moves
    rate = 200; % aiming for 200 Hz
    ros_rate = rosrate(rate);

    %%%% joint position move
    disp('---- Joint goal move');
    % move to 0 position
    p = r.get_state_joint_desired();
    joints_home = p;
    joints_home(:) = 0.0;
    if (strcmp(arm_name, 'ECM') || strncmp(arm_name, 'PSM', 3))
        joints_home(3) = 0.12;
    end
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

    disp('---- Joint direct move');
    % move to 0 position
    p = r.get_state_joint_desired();
    joints_home = p;
    joints_home(:) = 0.0;
    r.move_joint(joints_home);
    % wiggle first two joints, matlab index starts at 1
    amplitude = deg2rad(10.0);
    duration = 10.0; % seconds
    samples = duration * rate;
    % create a new goal starting with current position
    goal = p;
    reset(ros_rate);
    for i = 0:samples
    	goal(1) = p(1) + amplitude *  sin(i * deg2rad(360.0) / samples);
        goal(2) = p(2) + amplitude *  sin(i * deg2rad(360.0) / samples);
        r.move_joint(goal, false); % false -> do not interpolate
        waitfor(ros_rate);
    end

    %%%% cartesian position move
    disp('---- Cartesian goal move');
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

    disp('---- Cartesian direct move');
    % make sure tool tip is past RCM on ECM and PSMs
    n = r.robot_name;
    if (strcmp(n, 'ECM') || strncmp(n, 'PSM', 3))
        goal = joints_home;
        goal(3) = 0.12;
        r.move_joint(goal);
    end
    amplitude = 0.03; % 3 cm
    duration = 10.0; % seconds
    samples = duration * rate;
    % create a new goal starting with current position
    p = r.get_position_desired();
    goal = p;
    reset(ros_rate);
    for i = 0:samples
    	goal(1:2, 4) = p(1:2, 4) + amplitude *  sin(i * deg2rad(360.0) / samples);
        r.move(goal, false); % false -> do not interpolate
        waitfor(ros_rate);
    end

end
