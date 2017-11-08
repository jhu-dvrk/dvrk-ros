classdef console < handle
    % Class used to interface with ROS dVRK console topics and convert to useful
    % Matlab commands and properties.  To create a console interface:
    %   c = console();
    %   c
    %

    % settings that are not supposed to change after constructor
    properties (SetAccess = immutable)
        ros_namespace % namespace for this arm, should contain head/tail / (default is /dvrk/)
    end

    % only this class methods can view/modify
    properties (SetAccess = private)
        % subscribers
        teleop_scale_subscriber
        % publishers
        power_off_publisher
        power_on_publisher
        home_publisher
        teleop_enable_publisher
        teleop_set_scale_publisher
    end

    methods

        function self = console(namespace)
            % Create a console interface.  The namespace is
            % optional, default is /dvrk/.  It is provide for
            % configurations with multiple dVRK so one could have
            % /dvrkA/ and /dvrkB/
            if nargin == 0
                namespace = '/dvrk/console';
            end
            self.ros_namespace = namespace;

            % ----------- subscribers
            % teleop scale
            topic = strcat(self.ros_namespace, '/teleop/scale');
            self.teleop_scale_subscriber = ...
                rossubscriber(topic, rostype.std_msgs_Float32);

            % ----------- publishers
            % power off
            topic = strcat(self.ros_namespace, '/power_off');
            self.power_off_publisher = rospublisher(topic, rostype.std_msgs_Empty);

            % power on
            topic = strcat(self.ros_namespace, '/power_on');
            self.power_on_publisher = rospublisher(topic, rostype.std_msgs_Empty);

            % home
            topic = strcat(self.ros_namespace, '/home');
            self.home_publisher = rospublisher(topic, rostype.std_msgs_Empty);

            % teleop enable
            topic = strcat(self.ros_namespace, '/teleop/enable');
            self.teleop_enable_publisher = rospublisher(topic, rostype.std_msgs_Bool);

            % teleop set scale
            topic = strcat(self.ros_namespace, '/teleop/set_scale');
            self.teleop_set_scale_publisher = rospublisher(topic, rostype.std_msgs_Float32);
        end




        function delete(self)
        end


        function scale = teleop_get_scale(self)
           % Accessor used to retrieve the last teleop scale
           scale = self.teleop_scale_subscriber.LatestMessage.Data;
        end

        function power_off(self)
            message = rosmessage(self.power_off_publisher);
            send(self.power_off_publisher, message);
        end

        function power_on(self)
            message = rosmessage(self.power_on_publisher);
            send(self.power_on_publisher, message);
        end

        function home(self)
            message = rosmessage(self.home_publisher);
            send(self.home_publisher, message);
        end

        function teleop_start(self)
            message = rosmessage(self.teleop_enable_publisher);
            message.Data = true;
            send(self.teleop_enable_publisher, message);
        end

        function teleop_stop(self)
            message = rosmessage(self.teleop_enable_publisher);
            message.Data = false;
            send(self.teleop_enable_publisher, message);
        end

        function teleop_set_scale(self, scale)
            message = rosmessage(self.teleop_set_scale_publisher);
            message.Data = scale;
            send(self.teleop_set_scale_publisher, message);
        end

    end % methods

end % class
