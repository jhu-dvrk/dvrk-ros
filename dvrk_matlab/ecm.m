classdef ecm < arm
    % Class for ECM specific features

    methods

        function self = ecm(name)
            self@arm(name);
        end

        function result = insert_endoscope(self, depth)
            result = self.move_joint_one(depth, int8(3));
        end

        function result = dinsert_endoscope(self, depth)
            result = self.dmove_joint_one(depth, int8(3));
        end

    end % methods

end % class
