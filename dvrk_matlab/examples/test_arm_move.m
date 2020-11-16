% call methods to make sure they exist and don't trigger syntax errors
% this test program will make the arm move!
function test_arm_move(arm_name)
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

    % general settings
    rate = 200; % aiming for 200 Hz
    ros_rate = rosrate(rate);

    % move_jp
    disp('---- Joint move');
    % move to 0 position
    joints_home = r.setpoint_js();
    joints_home(:) = 0.0;
    if (strcmp(arm_name, 'ECM') || strncmp(arm_name, 'PSM', 3))
        joints_home(3) = 0.12;
    end
    r.move_jp(joints_home).wait();
    % wiggle first two joints, matlab index starts at 1
    amplitude = deg2rad(5.0);
    % first move
    start = r.setpoint_js();
    goal = start;
    goal(1:2) = amplitude;
    r.move_jp(goal).wait();
    % second move
    goal = start;
    goal(1:2) = -amplitude;
    r.move_jp(goal).wait();

    disp('---- Joint servo');
    % move to 0 position
    r.move_jp(joints_home).wait();
    % wiggle first two joints, matlab index starts at 1
    amplitude = deg2rad(10.0);
    duration = 10.0; % seconds
    samples = duration * rate;
    % create a new goal starting with current position
    start = r.setpoint_js();
    goal = start;
    reset(ros_rate);
    for i = 0:samples
        goal(1) = start(1) + amplitude * (1.0 - cos(i * deg2rad(360.0) / samples));
        goal(2) = start(2) + amplitude * (1.0 - cos(i * deg2rad(360.0) / samples));
        r.servo_jp(goal);
        waitfor(ros_rate);
    end

    % move_cp
    disp('---- Cartesian move');
    % move to 0 position
    r.move_jp(goal).wait();
    % start from current position
    start = r.setpoint_cp();
    amplitude = 0.03; % 3 cm
    % first move, translation is 4th column, x and y are element 1:2
    goal = start;
    goal(1:2, 4) = goal(1:2, 4) + amplitude;
    r.move_cp(goal).wait();
    % second move
    goal = start;
    goal(1:2, 4) = goal(1:2, 4) - amplitude;
    r.move_cp(goal).wait();
    % third move
    r.move_cp(start).wait();

    disp('---- Cartesian servo');
     % move to 0 position
    r.move_jp(joints_home).wait();
    % move in diagonal around a point
    amplitude = 0.03; % 3 cm
    duration = 10.0; % seconds
    samples = duration * rate;
    % create a new goal starting with current position
    start = r.setpoint_cp();
    goal = start;
    reset(ros_rate);
    for i = 0:samples
        % translation is 4th column, x and y are element 1:2
        goal(1:2, 4) = start(1:2, 4) + amplitude * (1.0 - cos(i * deg2rad(360.0) / samples));
        r.servo_cp(goal);
        waitfor(ros_rate);
    end

    % don't forget to cleanup
    disp('---- Delete arm class');
    delete(r);
end
