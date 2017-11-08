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
    %   r.get_position_desired()   % contains 4x4 homogeneous transform
    %   r.get_position_current()   % actual 4x4 for the reported position based
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
    end

    % only this class methods can view/modify
    properties (SetAccess = private)
        % subscribers
        robot_state_subscriber;
        goal_reached_subscriber;
        position_desired_subscriber;
        position_local_desired_subscriber;
        state_joint_desired_subscriber;
        position_current_subscriber;
        position_local_current_subscriber;
        twist_body_current_subscriber;
        wrench_body_current_subscriber;
        state_joint_current_subscriber;
        jacobian_spatial_subscriber;
        jacobian_body_subscriber;
        % publishers
        robot_state_publisher;
        position_goal_joint_publisher;
        position_joint_publisher;
        position_goal_publisher;
        position_publisher;
        wrench_body_orientation_absolute_publisher;
        wrench_body_publisher;
        gravity_compensation_publisher;
        % message placeholders
        std_msgs_Bool;
        std_msgs_String;
        sensor_msgs_JointState;
        geometry_msgs_Pose;
        geometry_msgs_Wrench;
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
            topic = strcat(self.ros_name, '/current_state');
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
            topic = strcat(self.ros_name, '/position_cartesian_desired');
            self.position_desired_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_PoseStamped);

            % position cartesian local desired
            topic = strcat(self.ros_name, '/position_cartesian_local_desired');
            self.position_local_desired_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_PoseStamped);

            % state joint desired
            topic = strcat(self.ros_name, '/state_joint_desired');
            self.state_joint_desired_subscriber = ...
                rossubscriber(topic, rostype.sensor_msgs_JointState);

            % position cartesian current
            topic = strcat(self.ros_name, '/position_cartesian_current');
            self.position_current_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_PoseStamped);

            % position cartesian local current
            topic = strcat(self.ros_name, '/position_cartesian_local_current');
            self.position_local_current_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_PoseStamped);

            % twist cartesian current
            topic = strcat(self.ros_name, '/twist_body_current');
            self.twist_body_current_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_TwistStamped);

            % wrench cartesian current
            topic = strcat(self.ros_name, '/wrench_body_current');
            self.wrench_body_current_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_WrenchStamped);

            % state joint current
            topic = strcat(self.ros_name, '/state_joint_current');
            self.state_joint_current_subscriber = ...
                rossubscriber(topic, rostype.sensor_msgs_JointState);

            % jacobian spatial
            topic = strcat(self.ros_name, '/jacobian_spatial');
            self.jacobian_spatial_subscriber = ...
                rossubscriber(topic, rostype.std_msgs_Float64MultiArray);

            % jacobian body
            topic = strcat(self.ros_name, '/jacobian_body');
            self.jacobian_body_subscriber = ...
                rossubscriber(topic, rostype.std_msgs_Float64MultiArray);

            % ----------- publishers
            % state
            topic = strcat(self.ros_name, '/set_desired_state');
            self.robot_state_publisher = rospublisher(topic, rostype.std_msgs_String);

            % position goal joint
            topic = strcat(self.ros_name, '/set_position_goal_joint');
            self.position_goal_joint_publisher = rospublisher(topic, ...
                                                              rostype.sensor_msgs_JointState);
            topic = strcat(self.ros_name, '/set_position_joint');
            self.position_joint_publisher = rospublisher(topic, ...
                                                         rostype.sensor_msgs_JointState);

            % position goal cartesian
            topic = strcat(self.ros_name, '/set_position_goal_cartesian');
            self.position_goal_publisher = rospublisher(topic, ...
                                                        rostype.geometry_msgs_Pose);
            topic = strcat(self.ros_name, '/set_position_cartesian');
            self.position_publisher = rospublisher(topic, ...
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

            % one time creation of messages to prevent lookup and creation at each call
            self.std_msgs_Bool = rosmessage(rostype.std_msgs_Bool);
            self.std_msgs_String = rosmessage(rostype.std_msgs_String);
            self.sensor_msgs_JointState = rosmessage(rostype.sensor_msgs_JointState);
            self.geometry_msgs_Pose = rosmessage(rostype.geometry_msgs_Pose);
            self.geometry_msgs_Wrench = rosmessage(rostype.geometry_msgs_Wrench);
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
            % the controller. We use a timer to synchronize the callback
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




        function seconds = ros_time_to_secs(~, stamp)
            % Convert awkward rostime into a single double
            seconds = double(stamp.Sec) + double(stamp.Nsec) * 10^-9;
        end

        function frame = ros_pose_to_frame(~, pose)
           % convert idiotic ROS message type to homogeneous transforms
           position = trvec2tform([pose.Position.X, pose.Position.Y, pose.Position.Z]);
           orientation = quat2tform([pose.Orientation.W, pose.Orientation.X, pose.Orientation.Y, pose.Orientation.Z]);
           frame = position * orientation;
        end

        function vector = ros_twist_to_vector(~, twist)
            vector = [twist.Linear.X,  twist.Linear.Y,  twist.Linear.Z, ...
                      twist.Angular.X, twist.Angular.Y, twist.Angular.Z];
        end

        function vector = ros_wrench_to_vector(~, wrench)
           % convert idiotic ROS message type to a single vector
           vector = [wrench.Force.X,  wrench.Force.Y,  wrench.Force.Z, ...
                     wrench.Torque.X, wrench.Torque.Y, wrench.Torque.Z];
        end

        function [frame, timestamp] = get_position_desired(self)
           % Accessor used to retrieve the last desired cartesian position
           frame = self.ros_pose_to_frame(self.position_desired_subscriber.LatestMessage.Pose);
           timestamp = self.ros_time_to_secs(self.position_desired_subscriber.LatestMessage.Header.Stamp);
        end

        function [frame, timestamp] = get_position_local_desired(self)
           % Accessor used to retrieve the last desired local cartesian position
           frame = self.ros_pose_to_frame(self.position_local_desired_subscriber.LatestMessage.Pose);
           timestamp = self.ros_time_to_secs(self.position_local_desired_subscriber.LatestMessage.Header.Stamp);
        end

        function [position, velocity, effort, timestamp] = get_state_joint_desired(self)
            % Accessor used to retrieve the last desired joint position/effort
            position = self.state_joint_desired_subscriber.LatestMessage.Position;
            velocity = self.state_joint_desired_subscriber.LatestMessage.Velocity;
            effort = self.state_joint_desired_subscriber.LatestMessage.Effort;
            timestamp = self.ros_time_to_secs(self.state_joint_desired_subscriber.LatestMessage.Header.Stamp);
        end

        function [frame, timestamp] = get_position_current(self)
           % Accessor used to retrieve the last current cartesian position
           frame = self.ros_pose_to_frame(self.position_current_subscriber.LatestMessage.Pose);
           timestamp = self.ros_time_to_secs(self.position_current_subscriber.LatestMessage.Header.Stamp);
        end

        function [frame, timestamp] = get_position_local_current(self)
           % Accessor used to retrieve the last current local cartesian position
           frame = self.ros_pose_to_frame(self.position_local_current_subscriber.LatestMessage.Pose);
           timestamp = self.ros_time_to_secs(self.position_local_current_subscriber.LatestMessage.Header.Stamp);
        end

        function [position, velocity, effort, timestamp] = get_state_joint_current(self)
            % Accessor used to retrieve the last measured joint position/velocity/effort
            position = self.state_joint_current_subscriber.LatestMessage.Position;
            velocity = self.state_joint_current_subscriber.LatestMessage.Velocity;
            effort = self.state_joint_current_subscriber.LatestMessage.Effort;
            timestamp = self.ros_time_to_secs(self.state_joint_current_subscriber.LatestMessage.Header.Stamp);
        end

        function [twist, timestamp] = get_twist_body_current(self)
            % Accessor used to retrieve the last measured cartesian velocity
            twist = self.ros_twist_to_vector(self.twist_body_current_subscriber.LatestMessage.Twist);
            timestamp = self.ros_time_to_secs(self.twist_body_current_subscriber.LatestMessage.Header.Stamp);
        end

        function [wrench, timestamp] = get_wrench_current(self)
            % Accessor used to retrieve the last measured cartesian wrench
            wrench = self.ros_wrench_to_vector(self.wrench_body_current_subscriber.LatestMessage.Wrench);
            timestamp = self.ros_time_to_secs(self.wrench_body_current_subscriber.LatestMessage.Header.Stamp);
        end

        function jacobian = get_jacobian_spatial(self)
            % Accessor used to retrieve the last jacobian
            msg = self.jacobian_spatial_subscriber.LatestMessage;
            jacobian = reshape(msg.Data, msg.Layout.Dim(2,1).Size, msg.Layout.Dim(1,1).Size)';
        end

        function jacobian = get_jacobian_body(self)
            % Accessor used to retrieve the last jacobian
            msg = self.jacobian_body_subscriber.LatestMessage;
            jacobian = reshape(msg.Data, msg.Layout.Dim(2,1).Size, msg.Layout.Dim(1,1).Size)';
        end



        function home(self)
            % Home the robot.  This method will request power on, calibrate
            % and then go to home position.   For a PSM, this method will
            % wait for the state READY.  This state can only be
            % reached if the user inserts the sterile adapter and the tool
            self.std_msgs_String.Data = 'READY';
            send(self.robot_state_publisher, ...
                 self.std_msgs_String);
            counter = 21; % up to 20 * 3 seconds transitions to get ready
            self.robot_state_timer.StartDelay = 3.0;
            done = false;
            while not(done)
                start(self.robot_state_timer);
                wait(self.robot_state_timer);
                counter = counter - 1;
                homed = strcmp(self.robot_state, 'READY');
                done = (counter == 0) | homed;
                if (not(done))
                    disp(strcat(self.robot_name, ...
                                ': waiting for state READY, ', ...
                                int2str(counter)))
                end
            end
            if not(homed)
                disp(strcat(self.robot_name, ...
                            ': failed to home, last state "', ...
                            self.robot_state, '"'))
            end
        end




        function result = move_joint(self, joint_values, interpolate)
            % Move to absolute joint value

            % default for interpolate is true
            if nargin == 2
                interpolate = true;
            end

            % check if the array provided has the right length
            if length(joint_values) == length(self.get_state_joint_current())
                % prepare the ROS message
                self.sensor_msgs_JointState.Position = joint_values;
                if interpolate
                    % reset goal reached value and timer
                    self.goal_reached = false;
                    start(self.goal_reached_timer);
                    % send message
                    send(self.position_goal_joint_publisher, ...
                         self.sensor_msgs_JointState);
                    % wait for timer to be interrupted by goal_reached
                    wait(self.goal_reached_timer);
                    result = self.goal_reached;
                else
                    % send message
                    send(self.position_joint_publisher, ...
                         self.sensor_msgs_JointState);
                     result = true;
                end
            else
                result = false;
                disp(strcat(self.robot_name, ...
                            [': move_joint, joint_values does not have right ' ...
                             'size, size of provided array is '], ...
                            int2str(length(joint_values))));
            end
        end




        function result = move(self, frame, interpolate)
            % Move to absolute cartesian frame

            % default for interpolate is true
            if nargin == 2
                interpolate = true;
            end

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

            % prepare the ROS message
            self.geometry_msgs_Pose.Position.X = frame(1, 4);
            self.geometry_msgs_Pose.Position.Y = frame(2, 4);
            self.geometry_msgs_Pose.Position.Z = frame(3, 4);
            quaternion = tform2quat(frame);
            self.geometry_msgs_Pose.Orientation.W = quaternion(1);
            self.geometry_msgs_Pose.Orientation.X = quaternion(2);
            self.geometry_msgs_Pose.Orientation.Y = quaternion(3);
            self.geometry_msgs_Pose.Orientation.Z = quaternion(4);

            if interpolate
                % reset goal reached value and timer
                self.goal_reached = false;
                start(self.goal_reached_timer);
                % send message
                send(self.position_goal_publisher, ...
                     self.geometry_msgs_Pose);
                % wait for timer to be interrupted by goal_reached
                wait(self.goal_reached_timer);
                result = self.goal_reached;
            else
                % send message
                send(self.position_publisher, ...
                     self.geometry_msgs_Pose);
                result = true;
            end
        end




        function result = dmove_joint_one(self, joint_value, joint_index, interpolate)
            % Move one single joint by increment, first argument is value
            % (radian or meter), second parameter is joint
            % index (index start at 1 for first joint).
            % Example: r.dmove_joint_one(-0.01, int8(3))

            % default for interpolate is true
            if nargin == 3
                interpolate = true;
            end


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
            goal = self.get_state_joint_desired();
            goal(joint_index) = goal(joint_index) + joint_value;
            result = self.move_joint(goal, interpolate);
        end




        function result = dmove_joint(self, joint_values, interpolate)
            % Move all joints by increment, joint value are provided in SI
            % units (radian or meter).  Example: r.dmove_joint([-0.01, -0.01, 0.0, 0.0, 0.0, 0.0, 0.0])

            % default for interpolate is true
            if nargin == 2
                interpolate = true;
            end

            if ~isfloat(joint_values)
                result = false;
                disp(strcat(self.robot_name, ...
                            ': dmove_joint, joint_values must be a real array'));
                return;
            end
            if length(joint_values) ~= length(self.get_state_joint_current())
                result = false;
                disp(strcat(self.robot_name, ...
                            [': dmove_joint, joint_values does not have right ' ...
                             'size, size of provided array is '], ...
                            int2str(length(joint_values))));
                return
            end
            goal = self.get_state_joint_desired();
            goal = goal + joint_values';
            result = self.move_joint(goal, interpolate);
        end




        function result = move_joint_one(self, joint_value, joint_index, interpolate)
            % Move one single joint, first argument value (radian or meter), second parameter is
            % is joint index (index start at 1 for first joint).
            % Example: r.move_joint_one(-0.01, int8(3))

            % default for interpolate is true
            if nargin == 3
                interpolate = true;
            end

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
            goal = self.get_state_joint_desired();
            goal(joint_index) = joint_value;
            result = self.move_joint(goal, interpolate);
        end




        function result = dmove_translation(self, translation, interpolate)
            % default for interpolate is true
            if nargin == 2
                interpolate = true;
            end

            % Move incrementaly in cartesian space
            translationH = trvec2tform(translation);
            goal = self.get_position_desired();
            goal(1, 4) = goal(1, 4) + translationH(1, 4);
            goal(2, 4) = goal(2, 4) + translationH(2, 4);
            goal(3, 4) = goal(3, 4) + translationH(3, 4);
            result = self.move(goal, interpolate);
            return
        end




        function result = move_translation(self, translation, interpolate)
            % default for interpolate is true
            if nargin == 2
                interpolate = true;
            end

            % Move to absolute position in cartesian space
            translationH = trvec2tform(translation);
            goal = self.get_position_desired();
            goal(1, 4) = translationH(1, 4);
            goal(2, 4) = translationH(2, 4);
            goal(3, 4) = translationH(3, 4);
            result = self.move(goal, interpolate);
            return
        end



        function result = set_wrench_body_orientation_absolute(self, ...
                                                              absolute)
            self.std_msgs_Bool.Data = absolute;
            % send message
            send(self.wrench_body_orientation_absolute_publisher, ...
                 self.std_msgs_Bool);
            result = true;
        end



        function result = set_wrench_body(self, wrench)
            % Set wrench body, expects an array of 6 elements

            % check if the array provided has the right length
            if (length(wrench) == 6)
                % prepare the ROS message
                self.geometry_msgs_Wrench.Force.X = wrench(1);
                self.geometry_msgs_Wrench.Force.Y = wrench(2);
                self.geometry_msgs_Wrench.Force.Z = wrench(3);
                self.geometry_msgs_Wrench.Torque.X = wrench(4);
                self.geometry_msgs_Wrench.Torque.Y = wrench(5);
                self.geometry_msgs_Wrench.Torque.Z = wrench(6);
                % send message
                send(self.wrench_body_publisher, ...
                     self.geometry_msgs_Wrench);
                result = true;
            else
                result = false;
                disp(strcat(self.robot_name, ...
                            ': set_wrench_body, wrench does not have right ', ...
                            'size, expecting 6 but size of provided array is ', ...
                            int2str(length(joint_values))));
            end
        end



        function result = set_gravity_compensation(self, ...
                                                   gravity)
            self.std_msgs_Bool.Data = gravity;
            % send message
            send(self.gravity_compensation_publisher, ...
                 self.std_msgs_Bool);
            result = true;
        end


    end % methods

end % class
