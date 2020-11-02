classdef ecm < dvrk.arm
    % Class for ECM specific features
    methods

        function self = ecm(name)
            self@dvrk.arm(name);
        end

        function [time] = insert_jp(self, depth)
            p = self.setpoint_js();
            p(3) = depth;
            time =  self.move_jp(p);
        end

    end

end
