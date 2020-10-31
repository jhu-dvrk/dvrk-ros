classdef psm < dvrk.arm
    % Class for PSM specific features
    % only this class methods can view/modify
    properties (SetAccess = protected)
        jaw;
        % publishers
        tool_present_publisher;
    end

    methods

        function self = psm(name)
            self@dvrk.arm(name);
            self.jaw = dvrk.psm_jaw(strcat(name, '/jaw'));
            % ----------- publishers
            topic = strcat(self.ros_namespace, '/set_tool_present');
            self.tool_present_publisher = rospublisher(topic, rostype.std_msgs_Bool);
        end

        function [time] = insert_jp(self, depth)
            p = self.setpoint_js();
            p(3) = depth;
            time =  self.move_jp(p);
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
