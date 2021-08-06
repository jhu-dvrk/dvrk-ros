% call methods to make sure they exist and don't trigger syntax errors
% this test program will make the arm move!
function test_move_wait(arm_name)
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

    % amplitude of motion
    amplitude = deg2rad(10.0);
    goal = r.setpoint_js();
    initial_position = goal(1);

    % move_jp
    disp('--> Testing the trajectory with wait()');

    tic;

    % first motion
    goal(1) = initial_position + amplitude;
    r.move_jp(goal).wait();

    % second motion
    goal(1) = initial_position - amplitude;
    r.move_jp(goal).wait();

    % third motion
    goal(1) = initial_position;
    r.move_jp(goal).wait();

    fprintf('--> Time for the full trajectory: ');
    toc

    % now with "busy" loop
    disp('--> Testing the trajectory with busy loop')

    counter = 0;

    tic;

    % first motion
    goal(1) = initial_position + amplitude;
    handle = r.move_jp(goal);
    while handle.is_busy()
        counter = counter + 1;
        fprintf('%g ', counter);
    end

    % second motion
    goal(1) = initial_position - amplitude;
    handle = r.move_jp(goal);
    fprintf('\n');
    while handle.is_busy()
        counter = counter + 1;
        fprintf('%g ', counter);
    end

    % third motion
    goal(1) = initial_position;
    handle = r.move_jp(goal);
    fprintf('\n');
    while handle.is_busy()
        counter = counter + 1;
        fprintf('%g ', counter);
    end

    fprintf('\n');

    fprintf('--> Time for the full trajectory: ');
    toc

    fprintf('--> You can change the trajectory velocity in the GUI using "%s", "Direct control" and lower the "100%%" factor. Then re-run this program.\n', ...
            arm_name)

    % don't forget to cleanup
    disp('---- Delete arm class');
    delete(r);
end
