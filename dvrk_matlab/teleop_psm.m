classdef teleop_psm < handle
    % Class used to interface with ROS dVRK teleop PSM topics and convert to useful
    % Matlab commands and properties.  To create a console interface:
    %   t = teleop_psm('MTMR_PSM1');
    %   t
    %

    % settings that are not supposed to change after constructor
    properties (SetAccess = immutable)
        ros_namespace % namespace for this arm, should contain head/tail / (default is /dvrk/)
        teleop_name   % name of teleop, e.g. MTMR_PSM1.  Must match ROS topics namespace
        ros_name      % full ROS namespace, i.e. ros_name + teleop_name (e.g. /dvrk/MTMR_PSM1)
    end

    % only this class methods can view/modify
    properties (SetAccess = private)
        % subscribers
        scale_subscriber
        % publishers
        set_desired_state_publisher
        set_scale_publisher
    end

    methods

        function self = teleop_psm(name, namespace)
            % Create a teleop interface.  The name must match the arm name
            % in ROS topics (test using rostopic list).  The namespace is
            % optional, default is /dvrk/.  It is provide for
            % configurations with multiple dVRK so one could have
            % /dvrkA/MTMR_PSM1 and /dvrkB/MTMR_PSM1
            if nargin == 1
                namespace = '/dvrk/';
            end
            self.ros_namespace = namespace;
            self.teleop_name = name;
            self.ros_name = strcat(self.ros_namespace, self.teleop_name);

            % ----------- subscribers
            % scale
            topic = strcat(self.ros_name, '/scale');
            self.scale_subscriber = ...
                rossubscriber(topic, rostype.std_msgs_Float32);

            % ----------- publishers
            % set scale
            topic = strcat(self.ros_name, '/set_scale');
            self.set_scale_publisher = rospublisher(topic, rostype.std_msgs_Float32);

            % desired state
            topic = strcat(self.ros_name, '/set_desired_state');
            self.set_desired_state_publisher = rospublisher(topic, rostype.std_msgs_String);

        end




        function delete(self)
        end


        function scale = get_scale(self)
           % Accessor used to retrieve the last teleop scale
           scale = self.scale_subscriber.LatestMessage.Data;
        end

        function set_scale(self, scale)
            message = rosmessage(self.set_scale_publisher);
            message.Data = scale;
            send(self.set_scale_publisher, message);
        end

        function enable(self)
            message = rosmessage(self.set_desired_state_publisher);
            message.Data = 'ENABLED';
            send(self.set_desired_state_publisher, message);
        end

        function disable(self)
            message = rosmessage(self.set_desired_state_publisher);
            message.Data = 'DISABLED';
            send(self.set_desired_state_publisher, message);
        end

    end % methods

end % class
