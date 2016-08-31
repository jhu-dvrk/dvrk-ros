classdef psm < arm
    % Class for PSM specific features

    % only this class methods can view/modify
    properties (SetAccess = private)
        % publishers
        jaw_position_publisher
        tool_present_publisher
    end

    methods

        function self = psm(name)
            self@arm(name);

            % ----------- publishers
            topic = strcat(self.ros_name, '/set_jaw_position');
            self.jaw_position_publisher = rospublisher(topic, rostype.std_msgs_Float32);

            topic = strcat(self.ros_name, '/set_tool_present');
            self.tool_present_publisher = rospublisher(topic, rostype.std_msgs_Bool);

        end


        function result = move_jaw(self, jaw_angle)
            % Set the jaw angle
            if self.set_state('DVRK_POSITION_GOAL_CARTESIAN')
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
            else
                % unable to set the desired state
                % set_state should already provide a message
                result = false;
            end
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

        function result = set_tool_present(self, ...
					   tp)
            tp_message = rosmessage(self.tool_present_publisher);
            tp_message.Data = tp;
            % send message
            send(self.tool_present_publisher, ...
                 tp_message);
            result = true;
        end

    end % methods

end % class
