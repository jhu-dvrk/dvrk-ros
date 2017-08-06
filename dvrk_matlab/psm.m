classdef psm < arm
    % Class for PSM specific features

    % only this class methods can view/modify
    properties (SetAccess = private)
        % subscribers
        state_jaw_desired_subscriber
        state_jaw_current_subscriber
        % publishers
        jaw_position_publisher
        tool_present_publisher
    end

    methods

        function self = psm(name)
            self@arm(name);

            % ----------- subscribers
            % state jaw desired
            topic = strcat(self.ros_name, '/state_jaw_desired');
            self.state_jaw_desired_subscriber = ...
                rossubscriber(topic, rostype.sensor_msgs_JointState);

            % state jaw current
            topic = strcat(self.ros_name, '/state_jaw_current');
            self.state_jaw_current_subscriber = ...
                rossubscriber(topic, rostype.sensor_msgs_JointState);

            % ----------- publishers
            topic = strcat(self.ros_name, '/set_jaw_position');
            self.jaw_position_publisher = rospublisher(topic, rostype.std_msgs_Float32);

            topic = strcat(self.ros_name, '/set_tool_present');
            self.tool_present_publisher = rospublisher(topic, rostype.std_msgs_Bool);

        end


        function [position, velocity, effort, timestamp] = get_state_jaw_desired(self)
            % Accessor used to retrieve the last desired jaw position/effort
            position = self.state_jaw_desired_subscriber.LatestMessage.Position;
            velocity = self.state_jaw_desired_subscriber.LatestMessage.Velocity;
            effort = self.state_jaw_desired_subscriber.LatestMessage.Effort;
            timestamp = self.ros_time_to_secs(self.state_jaw_desired_subscriber.LatestMessage.Header.Stamp);
        end


        function [position, velocity, effort, timestamp] = get_state_jaw_current(self)
            % Accessor used to retrieve the last current jaw position/effort
            position = self.state_jaw_current_subscriber.LatestMessage.Position;
            velocity = self.state_jaw_current_subscriber.LatestMessage.Velocity;
            effort = self.state_jaw_current_subscriber.LatestMessage.Effort;
            timestamp = self.ros_time_to_secs(self.state_jaw_current_subscriber.LatestMessage.Header.Stamp);
        end


        function result = move_jaw(self, jaw_angle)
            % Set the jaw angle
            % prepare the ROS message
            jaw_message = rosmessage(self.jaw_position_publisher);
            jaw_message.Data = jaw_angle;
            % reset goal reached value and timer
            self.goal_reached = false;
            start(self.goal_reached_timer);
            % send message
            send(self.jaw_position_publisher, ...
                 jaw_message)
            % wait for timer to be interrupted by goal_reached
            wait(self.goal_reached_timer);
            result = self.goal_reached;
        end


        function result = close_jaw(self)
            result = self.move_jaw(-20.0 * pi / 180.0);
        end


        function result = open_jaw(self)
            result = self.move_jaw(80.0 * pi / 180.0);
        end


        function result = insert_tool(self, depth)
            result = self.move_joint_one(depth, int8(3));
        end


        function result = dinsert_tool(self, depth)
            result = self.dmove_joint_one(depth, int8(3));
        end

        function result = set_tool_present(self, tp)
            tp_message = rosmessage(self.tool_present_publisher);
            tp_message.Data = tp;
            % send message
            send(self.tool_present_publisher, ...
                 tp_message);
            result = true;
        end

    end % methods

end % class
