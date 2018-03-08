classdef suj < handle
    % Class used to interface with ROS dVRK console topics and convert to useful
    % Matlab commands and properties.  To create a robot interface:
    %   r = suj('PSM1');
    %   r
    %
    % In general, the word `cartesian` is omitted.  When using joint space,
    % add the word `joint`.  `move` stands for moves using absolute
    % positions while `dmove` are always relative to the current desired
    % position as reported by the dVRK C++ console application (i.e. last desired command).
    %   r.get_position_current()   % actual 4x4 for the reported position based
    %   on encoders

    % settings that are not supposed to change after constructor
    properties (SetAccess = immutable)
        ros_namespace % namespace for this arm, should contain head/tail / (default is /dvrk/SUJ)
        robot_name    % name of robot, e.g. PSM1, ECM.  Must match ROS topics namespace
        ros_name      % full ROS namespace, i.e. ros_name + robot_name (e.g. /dvrk/SUJ/PSM1)
    end

    % only this class methods can view/modify
    properties (SetAccess = private)
        % subscribers
        position_current_subscriber
        position_local_current_subscriber
    end

    methods

        function self = suj(name, namespace)
            % Create a robot interface.  The name must match the suj name
            % in ROS topics (test using rostopic list).  The namespace is
            % optional, default is /dvrk/SUJ.  It is provide for
            % configurations with multiple dVRK so one could have
            % /dvrkA/SUJ/PSM1 and /dvrkB/SUJ/PSM1
            if nargin == 1
                namespace = '/dvrk/SUJ/';
            end
            self.ros_namespace = namespace;
            self.robot_name = name;
            self.ros_name = strcat(self.ros_namespace, self.robot_name);

            % ----------- subscribers

            % position cartesian current
            topic = strcat(self.ros_name, '/position_cartesian_current');
            self.position_current_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_PoseStamped);

            % position cartesian local current
            topic = strcat(self.ros_name, '/position_cartesian_local_current');
            self.position_local_current_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_PoseStamped);

        end


        function delete(self)
            % hack to disable callbacks from subscribers
            % there might be a better way to remove the subscriber itself
            self.position_current_subscriber.NewMessageFcn = @(a, b, c)[];
            self.position_local_current_subscriber.NewMessageFcn = @(a, b, c)[];
        end



        function seconds = ros_time_to_secs(~,pose_msg)
            % Convert awkward rostime into a single double
            seconds = double(pose_msg.Header.Stamp.Sec)+double(pose_msg.Header.Stamp.Nsec)*10^-9;
        end

        function frame = ros_pose_to_frame(~,pose_msg)
           % convert idiotic ROS message type to homogeneous transforms
           position = trvec2tform([pose_msg.Pose.Position.X, pose_msg.Pose.Position.Y, pose_msg.Pose.Position.Z]);
           orientation = quat2tform([pose_msg.Pose.Orientation.W, pose_msg.Pose.Orientation.X, pose_msg.Pose.Orientation.Y, pose_msg.Pose.Orientation.Z]);
           frame = position*orientation;
        end

        function [frame, timestamp] = get_position_current(self)
           % Accessor used to retrieve the last current cartesian position
           msg = self.position_current_subscriber.LatestMessage;
           timestamp = self.ros_time_to_secs(msg);
           frame = self.ros_pose_to_frame(msg);
        end

        function [frame, timestamp] = get_position_local_current(self)
           % Accessor used to retrieve the last current local cartesian position
           msg = self.position_local_current_subscriber.LatestMessage;
           frame = self.ros_pose_to_frame(msg);
           timestamp = self.ros_time_to_secs(msg);
        end

    end % methods

end % class
