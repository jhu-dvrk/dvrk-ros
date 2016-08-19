classdef arm < handle
    % Class used to interface with ROS dVRK console topics and convert to useful
    % Matlab commands and properties.  To create a robot interface:
    %   r = arm('PSM1');
    %   r
    %
    % To home and check current state:
    %   r.home();
    %
    % In general, the word `cartesian` is omitted.  When using joint space,
    % add the word `joint`.  `move` stands for moves using absolute
    % positions while `dmove` are always relative to the current desired
    % position as reported by the dVRK C++ console application (i.e. last desired command).
    %   r.position_desired   % contains 4x4 homogeneous transform
    %   r.position_current   % actual 4x4 for the reported position based
    %   on encoders
    %
    % To move a single joint, be careful with radians and meters.  Also,
    % indices start at 1 in Matlab while C/C++ and Python start at 0.
    %   r.dmove_joint_one(-0.01, int8(3))
    %
    % To move all joints (radians and meters)
    %   r.dmove_joint([-0.01, -0.01, 0.0, 0.0, 0.0, 0.0, 0.0])
    %
    % To move in cartesian space, translation only:
    %   v = [0.0, 0.0, 0.01]
    %   r.dmove_translation(-v)

    % settings that are not supposed to change after constructor
    properties (SetAccess = immutable)
        ros_namespace % namespace for this arm, should contain head/tail / (default is /dvrk/)
        robot_name    % name of robot, e.g. PSM1, ECM.  Must match ROS topics namespace
        ros_name      % full ROS namespace, i.e. ros_name + robot_name (e.g. /dvrk/MTML)
    end

    % values set by this class, can be read by others
    properties (SetAccess = protected)
        robot_state             % Internal robot state, string corresponding to C++ state
        robot_state_timer       % Timer used to wait for new robot states
        goal_reached            % Event to indicate if trajectory goal is reached
        goal_reached_timer      % Timer to block on trajectory

        position_desired        % Last received desired cartesian position
        position_current        % Last received current cartesian position
        position_local_desired  % Last received desired cartesian position
        position_local_current  % Last received current cartesian position
        twist_body_current      % Last received current cartesian twist body
        wrench_body_current     % Last received current cartesian wrench body
        position_joint_desired  % Last received desired joint position (PID input)
        effort_joint_desired    % Last received desired joint effort (PID output)
        position_joint_current  % Last received current joint position
        velocity_joint_current  % Last received current joint velocity
        effort_joint_current    % Last received current joint effort
    end

    % only this class methods can view/modify
    properties (SetAccess = private)
        % subscribers
        robot_state_subscriber
        goal_reached_subscriber
        position_desired_subscriber
        position_local_desired_subscriber
        state_joint_desired_subscriber
        position_current_subscriber
        position_local_current_subscriber
        twist_body_current_subscriber
        wrench_body_current_subscriber
        state_joint_current_subscriber
        % publishers
        robot_state_publisher
        position_goal_joint_publisher
        position_goal_publisher
        wrench_body_orientation_absolute_publisher
        wrench_body_publisher
        gravity_compensation_publisher
    end

    methods

        function self = arm(name, namespace)
            % Create a robot interface.  The name must match the arm name
            % in ROS topics (test using rostopic list).  The namespace is
            % optional, default is /dvrk/.  It is provide for
            % configurations with multiple dVRK so one could have
            % /dvrkA/PSM1 and /dvrkB/PSM1
            if nargin == 1
                namespace = '/dvrk/';
            end
            self.ros_namespace = namespace;
            self.robot_name = name;
            self.ros_name = strcat(self.ros_namespace, self.robot_name);

            % ----------- subscribers
            % state
            topic = strcat(self.ros_name, '/robot_state');
            self.robot_state_subscriber = ...
                rossubscriber(topic, rostype.std_msgs_String);
            self.robot_state_subscriber.NewMessageFcn = ...
                @(sub, data)self.robot_state_cb(sub, data);
            % timer for robot state event
            self.robot_state_timer = timer('ExecutionMode', 'singleShot', ...
                                           'Name', strcat(self.ros_name, '_robot_state'), ...
                                           'ObjectVisibility', 'off');
            self.robot_state_timer.TimerFcn = { @self.robot_state_timout };

            % goal reached
            topic = strcat(self.ros_name, '/goal_reached');
            self.goal_reached_subscriber = ...
                rossubscriber(topic, rostype.std_msgs_Bool);
            self.goal_reached_subscriber.NewMessageFcn = ...
                @(sub, data)self.goal_reached_cb(sub, data);
            % timer for goal reached event
            self.goal_reached_timer = timer('ExecutionMode', 'singleShot', ...
                                           'Name', strcat(self.ros_name, '_goal_reached'), ...
                                           'ObjectVisibility', 'off', ...
                                           'StartDelay', 5.0); % 5s to complete trajectory

            self.goal_reached_timer.TimerFcn = { @self.goal_reached_timout };

            % position cartesian desired
            self.position_desired = [];
            topic = strcat(self.ros_name, '/position_cartesian_desired');
            self.position_desired_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_PoseStamped);
            self.position_desired_subscriber.NewMessageFcn = ...
                @(sub, data)self.position_desired_cb(sub, data);

            % position cartesian local desired
            self.position_local_desired = [];
            topic = strcat(self.ros_name, '/position_cartesian_local_desired');
            self.position_local_desired_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_PoseStamped);
            self.position_local_desired_subscriber.NewMessageFcn = ...
                @(sub, data)self.position_local_desired_cb(sub, data);

            % state joint desired
            self.position_joint_desired = [];
            self.effort_joint_desired = [];
            topic = strcat(self.ros_name, '/state_joint_desired');
            self.state_joint_desired_subscriber = ...
                rossubscriber(topic, rostype.sensor_msgs_JointState);
            self.state_joint_desired_subscriber.NewMessageFcn = ...
                @(sub, data)self.state_joint_desired_cb(sub, data);

            % position cartesian current
            self.position_current = [];
            topic = strcat(self.ros_name, '/position_cartesian_current');
            self.position_current_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_PoseStamped);
            self.position_current_subscriber.NewMessageFcn = ...
                @(sub, data)self.position_current_cb(sub, data);

            % position cartesian local current
            self.position_local_current = [];
            topic = strcat(self.ros_name, '/position_cartesian_local_current');
            self.position_local_current_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_PoseStamped);
            self.position_local_current_subscriber.NewMessageFcn = ...
                @(sub, data)self.position_local_current_cb(sub, data);

            % twist cartesian current
            self.twist_body_current = [];
            topic = strcat(self.ros_name, '/twist_body_current');
            self.twist_body_current_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_TwistStamped);
            self.twist_body_current_subscriber.NewMessageFcn = ...
                @(sub, data)self.twist_body_current_cb(sub, data);

            % wrench cartesian current
            self.wrench_body_current = [];
            topic = strcat(self.ros_name, '/wrench_body_current');
            self.wrench_body_current_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_WrenchStamped);
            self.wrench_body_current_subscriber.NewMessageFcn = ...
                @(sub, data)self.wrench_body_current_cb(sub, data);

            % state joint current
            self.position_joint_current = [];
            self.velocity_joint_current = [];
            self.effort_joint_current = [];
            topic = strcat(self.ros_name, '/state_joint_current');
            self.state_joint_current_subscriber = ...
                rossubscriber(topic, rostype.sensor_msgs_JointState);
            self.state_joint_current_subscriber.NewMessageFcn = ...
                @(sub, data)self.state_joint_current_cb(sub, data);

            % ----------- publishers
            % state
            topic = strcat(self.ros_name, '/set_robot_state');
            self.robot_state_publisher = rospublisher(topic, rostype.std_msgs_String);

            % position goal joint
            topic = strcat(self.ros_name, '/set_position_goal_joint');
            self.position_goal_joint_publisher = rospublisher(topic, ...
                                                              rostype.sensor_msgs_JointState);


            % position goal cartesian
            topic = strcat(self.ros_name, '/set_position_goal_cartesian');
            self.position_goal_publisher = rospublisher(topic, ...
                                                        rostype.geometry_msgs_Pose);


            % wrench cartesian
            topic = strcat(self.ros_name, '/set_wrench_body_orientation_absolute');
            self.wrench_body_orientation_absolute_publisher = ...
                rospublisher(topic, rostype.std_msgs_Bool);

            topic = strcat(self.ros_name, '/set_wrench_body');
            self.wrench_body_publisher = rospublisher(topic, ...
                                                      rostype.geometry_msgs_Wrench);

            % gravity compensation
            topic = strcat(self.ros_name, '/set_gravity_compensation');
            self.gravity_compensation_publisher = rospublisher(topic, ...
                                                              rostype.std_msgs_Bool);
        end




        function delete(self)
            % delete all timers
            delete(self.robot_state_timer);
            delete(self.goal_reached_timer);

            % hack to disable callbacks from subscribers
            % there might be a better way to remove the subscriber itself
            self.robot_state_subscriber.NewMessageFcn = @(a, b, c)[];
            self.goal_reached_subscriber.NewMessageFcn = @(a, b, c)[];
            self.position_desired_subscriber.NewMessageFcn = @(a, b, c)[];
            self.position_local_desired_subscriber.NewMessageFcn = @(a, b, c)[];
            self.state_joint_desired_subscriber.NewMessageFcn = @(a, b, c)[];
            self.position_current_subscriber.NewMessageFcn = @(a, b, c)[];
            self.position_local_current_subscriber.NewMessageFcn = @(a, b, c)[];
            self.state_joint_current_subscriber.NewMessageFcn = @(a, b, c)[];
        end




        function robot_state_cb(self, ~, data) % second argument is subscriber, not used
            % Method used internally when a new robot_state is published by
            % the controller.  We use a timer to synchronize the callback
            % and user code.
            self.robot_state = data.Data;
            stop(self.robot_state_timer);
        end

        function robot_state_timout(self, ~, ~) % second parameter is timer, third is this function
            disp(strcat(self.robot_name, ': timeout robot state, current state is "', self.robot_state, '"'));
        end

        function goal_reached_cb(self, ~, data) % second argument is subscriber, not used
            % Method used internally when a new goal_reached is published
            % by the controller.  We use a timer to synchronize the callback and user code.
            self.goal_reached = data.Data;
            stop(self.goal_reached_timer);
        end

        function goal_reached_timout(self, ~, ~) % second parameter is timer, third is this function
            disp(strcat(self.robot_name, ': timeout robot state, current state is "', self.goal_reached, '"'));
        end

        function position_desired_cb(self, ~, pose) % second argument is subscriber, not used
            % Callback used to retrieve the last desired cartesian position
            % published and store as property position_desired

            % convert idiotic ROS message type to homogeneous transforms
            position = trvec2tform([pose.Pose.Position.X, pose.Pose.Position.Y, pose.Pose.Position.Z]);
            orientation = quat2tform([pose.Pose.Orientation.W, pose.Pose.Orientation.X, pose.Pose.Orientation.Y, pose.Pose.Orientation.Z]);
            % combine position and orientation
            self.position_desired = position * orientation;
        end

        function position_local_desired_cb(self, ~, pose) % second argument is subscriber, not used
            % Callback used to retrieve the last desired cartesian position
            % published and store as property position_local_desired

            % convert idiotic ROS message type to homogeneous transforms
            position = trvec2tform([pose.Pose.Position.X, pose.Pose.Position.Y, pose.Pose.Position.Z]);
            orientation = quat2tform([pose.Pose.Orientation.W, pose.Pose.Orientation.X, pose.Pose.Orientation.Y, pose.Pose.Orientation.Z]);
            % combine position and orientation
            self.position_local_desired = position * orientation;
        end

        function state_joint_desired_cb(self, ~, jointState) % second argument is subscriber, not used
            % Callback used to retrieve the last desired joint
            % position/effort published and store as property position/effort_joint_desired
            self.position_joint_desired = jointState.Position;
            self.effort_joint_desired = jointState.Effort;
        end

        function position_current_cb(self, ~, pose) % second argument is subscriber, not used
            % Callback used to retrieve the last measured cartesian
            % position published and store as property position_current

            % convert idiotic ROS message type to homogeneous transforms
            position = trvec2tform([pose.Pose.Position.X, pose.Pose.Position.Y, pose.Pose.Position.Z]);
            orientation = quat2tform([pose.Pose.Orientation.W, pose.Pose.Orientation.X, pose.Pose.Orientation.Y, pose.Pose.Orientation.Z]);
            % combine position and orientation
            self.position_current = position * orientation;
        end

        function position_local_current_cb(self, ~, pose) % second argument is subscriber, not used
            % Callback used to retrieve the last measured cartesian
            % position published and store as property position_local_current

            % convert idiotic ROS message type to homogeneous transforms
            position = trvec2tform([pose.Pose.Position.X, pose.Pose.Position.Y, pose.Pose.Position.Z]);
            orientation = quat2tform([pose.Pose.Orientation.W, pose.Pose.Orientation.X, pose.Pose.Orientation.Y, pose.Pose.Orientation.Z]);
            % combine position and orientation
            self.position_local_current = position * orientation;
        end

        function twist_body_current_cb(self, ~, twist) % second argument is subscriber, not used
            % Callback used to retrieve the last measured cartesian
            % twist published and store as property twist_body_current

            % convert idiotic ROS message type to a single vector
            self.twist_body_current = [twist.Twist.Linear.X, twist.Twist.Linear.Y, twist.Twist.Linear.Z, ...
                                       twist.Twist.Angular.X, twist.Twist.Angular.Y, twist.Twist.Angular.Z];
        end

        function wrench_body_current_cb(self, ~, wrench) % second argument is subscriber, not used
            % Callback used to retrieve the last measured cartesian
            % position published and store as property position_current

            % convert idiotic ROS message type to a single vector
            self.wrench_body_current = [wrench.Wrench.Force.X, wrench.Wrench.Force.Y, wrench.Wrench.Force.Z, ...
                                        wrench.Wrench.Torque.X, wrench.Wrench.Torque.Y, wrench.Wrench.Torque.Z];
        end

        function state_joint_current_cb(self, ~, jointState) % second argument is subscriber, not used
            % Callback used to retrieve the last measured joint
            % position/velocity/effort
            % published and store as property position/velocity/effort_joint_current
            self.position_joint_current = jointState.Position;
            self.velocity_joint_current = jointState.Velocity;
            self.effort_joint_current = jointState.Effort;
        end




        function result = set_state(self, state_as_string)
            % Set the robot state.  For homing, used the robot.home()
            % method instead.  This is used internally to switch between
            % the joint/cartesian and direct/trajectory modes

            % first check that state is different
            if strcmp(state_as_string, self.robot_state)
                result = true;
                return
            end
            % else, prepare message and send
            message = rosmessage(self.robot_state_publisher);
            message.Data = state_as_string;
             % 5 seconds should be plenty for most state changes
            self.robot_state_timer.StartDelay = 5.0;
            start(self.robot_state_timer);
            send(self.robot_state_publisher, message);
            wait(self.robot_state_timer);
            % after waiting, check what is current state to make sure it
            % changed
            result = strcmp(self.robot_state, state_as_string);
            if not(result)
                disp(strcat(self.robot_name, ...
                            ': failed to reach state "', ...
                            state_as_string, ...
                            '", current state is "', ...
                            self.robot_state, ...
                            '"'));
            end
        end




        function home(self)
            % Home the robot.  This method will request power on, calibrate
            % and then go to home position.   For a PSM, this method will
            % wait for the state DVRK_READY.  This state can only be
            % reached if the user inserts the sterile adapter and the tool
            message = rosmessage(self.robot_state_publisher);
            message.Data = 'Home';
            send(self.robot_state_publisher, message);
            counter = 21; % up to 20 * 3 seconds transitions to get ready
            self.robot_state_timer.StartDelay = 3.0;
            done = false;
            while not(done)
                start(self.robot_state_timer);
                wait(self.robot_state_timer);
                counter = counter - 1;
                homed = strcmp(self.robot_state, 'DVRK_READY');
                done = (counter == 0) | homed;
                if (not(done))
                    disp(strcat(self.robot_name, ...
                                ': waiting for state DVRK_READY, ', ...
                                int2str(counter)))
                end
            end
            if not(homed)
                disp(strcat(self.robot_name, ...
                            ': failed to home, last state "', ...
                            self.robot_state, '"'))
            end
        end




        function result = move_joint(self, joint_values)
            % Move to absolute joint value using trajectory
            % generator

            % check if the array provided has the right length
            if length(joint_values) == length(self.position_joint_current)
                if self.set_state('DVRK_POSITION_GOAL_JOINT')
                    % prepare the ROS message
                    joint_message = rosmessage(self.position_goal_joint_publisher);
                    joint_message.Position = joint_values;
                    % reset goal reached value and timer
                    self.goal_reached = false;
                    start(self.goal_reached_timer);
                    % send message
                    send(self.position_goal_joint_publisher, ...
                         joint_message);
                    % wait for timer to be interrupted by goal_reached
                    wait(self.goal_reached_timer);
                    result = self.goal_reached;
                else
                    % unable to set the desired state
                    % set_state should already provide a message
                    result = false;
                end
            else
                result = false;
                disp(strcat(self.robot_name, ...
                            [': move_joint, joint_values does not have right ' ...
                             'size, size of provided array is '], ...
                            int2str(length(joint_values))));
            end
        end




        function result = move(self, frame)
            % Move to absolute cartesian frame using trajectory
            % generator

            if ~isreal(frame)
                result = false;
                disp(strcat(self.robot_name, ...
                            ': move, input must be an array or real numbers'));
               return
            end

            if ~ismatrix(frame)
                result = false;
                disp(strcat(self.robot_name, ...
                            ': move, input must be a matrix'));
               return
            end

            [nbRows, nbCols] = size(frame);
            if (nbRows ~= 4) || (nbCols ~=4)
                result = false;
                disp(strcat(self.robot_name, ...
                            ': move, input must be a 4x4 matrix'));
               return
            end

            % actual move
            if self.set_state('DVRK_POSITION_GOAL_CARTESIAN')
                % prepare the ROS message
                pose_message = rosmessage(self.position_goal_publisher);
                % convert to ROS idiotic data type
                pose_message.Position.X = frame(1, 4);
                pose_message.Position.Y = frame(2, 4);
                pose_message.Position.Z = frame(3, 4);
                quaternion = tform2quat(frame);
                pose_message.Orientation.W = quaternion(1);
                pose_message.Orientation.X = quaternion(2);
                pose_message.Orientation.Y = quaternion(3);
                pose_message.Orientation.Z = quaternion(4);
                % reset goal reached value and timer
                self.goal_reached = false;
                start(self.goal_reached_timer);
                % send message
                send(self.position_goal_publisher, ...
                     pose_message);
                % wait for timer to be interrupted by goal_reached
                wait(self.goal_reached_timer);
                result = self.goal_reached;
            else
                % unable to set the desired state
                % set_state should already provide a message
                result = false;
            end
        end




        function result = dmove_joint_one(self, joint_value, joint_index)
            % Move one single joint by increment, first argument is value
            % (radian or meter), second parameter is joint
            % index (index start at 1 for first joint).
            % Example: r.dmove_joint_one(-0.01, int8(3))
            if ~isinteger(joint_index)
                result = false;
                disp(strcat(self.robot_name, ...
                            ': dmove_joint_one, joint_index must be a real numbers (use int8(index) to create index)'));
                return;
            end
            if ~isfloat(joint_value)
                result = false;
                disp(strcat(self.robot_name, ...
                            ': dmove_joint_one, joint_value must be a real number'));
                return;
            end
            goal = self.position_joint_desired;
            goal(joint_index) = goal(joint_index) + joint_value;
            result = self.move_joint(goal);
        end




        function result = dmove_joint(self, joint_values)
            % Move all joints by increment, joint value are provided in SI
            % units (radian or meter).  Example: r.dmove_joint([-0.01, -0.01, 0.0, 0.0, 0.0, 0.0, 0.0])
            if ~isfloat(joint_values)
                result = false;
                disp(strcat(self.robot_name, ...
                            ': dmove_joint, joint_values must be a real array'));
                return;
            end
            if length(joint_values) ~= length(self.position_joint_current)
                result = false;
                disp(strcat(self.robot_name, ...
                            [': dmove_joint, joint_values does not have right ' ...
                             'size, size of provided array is '], ...
                            int2str(length(joint_values))));
                return
            end
            goal = self.position_joint_desired;
            goal = goal + joint_values';
            result = self.move_joint(goal);
        end




        function result = move_joint_one(self, joint_value, joint_index)
            % Move one single joint, first argument value (radian or meter), second parameter is
            % is joint index (index start at 1 for first joint).
            % Example: r.move_joint_one(-0.01, int8(3))
            if ~isinteger(joint_index)
                result = false;
                disp(strcat(self.robot_name, ...
                            ': move_joint_one, joint_index must be a real numbers (use int8(index) to create index)'));
                return;
            end
            if ~isfloat(joint_value)
                result = false;
                disp(strcat(self.robot_name, ...
                            ': move_joint_one, joint_value must be a real number'));
                return;
            end
            goal = self.position_joint_desired;
            goal(joint_index) = joint_value;
            result = self.move_joint(goal);
        end




        function result = dmove_translation(self, translation)
            % Move incrementaly in cartesian space
            translationH = trvec2tform(translation);
            goal = self.position_desired;
            goal(1, 4) = goal(1, 4) + translationH(1, 4);
            goal(2, 4) = goal(2, 4) + translationH(2, 4);
            goal(3, 4) = goal(3, 4) + translationH(3, 4);
            result = self.move(goal);
            return
        end




        function result = move_translation(self, translation)
            % Move to absolute position in cartesian space
            translationH = trvec2tform(translation);
            goal = self.position_desired;
            goal(1, 4) = translationH(1, 4);
            goal(2, 4) = translationH(2, 4);
            goal(3, 4) = translationH(3, 4);
            result = self.move(goal);
            return
        end



        function result = set_wrench_body_orientation_absolute(self, ...
                                                              absolute)
            orientation_message = rosmessage(self.wrench_body_orientation_absolute_publisher);
            orientation_message.Data = absolute;
            % send message
            send(self.wrench_body_orientation_absolute_publisher, ...
                 orientation_message);
            result = true;
        end



        function result = set_wrench_body(self, wrench)
            % Set wrench body, expects an array of 6 elements

            % check if the array provided has the right length
            if (length(wrench) == 6)
                if self.set_state('DVRK_EFFORT_CARTESIAN')
                    % prepare the ROS message
                    wrench_message = rosmessage(self.wrench_body_publisher);
                    wrench_message.Force.X = wrench(1);
                    wrench_message.Force.Y = wrench(2);
                    wrench_message.Force.Z = wrench(3);
                    wrench_message.Torque.X = wrench(4);
                    wrench_message.Torque.Y = wrench(5);
                    wrench_message.Torque.Z = wrench(6);
                    % send message
                    send(self.wrench_body_publisher, ...
                         wrench_message);
                    result = true;
                else
                    % unable to set the desired state
                    % set_state should already provide a message
                    result = false;
                end
            else
                result = false;
                disp(strcat(self.robot_name, [': set_wrench_body, wrench does not have right '], ...
                            ['size, expecting 6 but size of provided ' ...
                             'array is '], int2str(length(joint_values))));
            end
        end



        function result = set_gravity_compensation(self, ...
                                                   gravity)
            gravity_message = rosmessage(self.gravity_compensation_publisher);
            gravity_message.Data = gravity;
            % send message
            send(self.gravity_compensation_publisher, ...
                 gravity_message);
            result = true;
        end


    end % methods

end % class
