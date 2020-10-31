classdef arm_cf < dynamicprops
    properties (Access = protected)
        crtk_utils;
        ros_namespace;
    end

    methods

        function self = arm_cf(ros_namespace)
            self.ros_namespace = ros_namespace;
            self.crtk_utils = crtk.utils(self, ros_namespace);
            self.crtk_utils.add_measured_cf();
            self.crtk_utils.add_servo_cf();
        end

        function delete(self)
            delete(self.crtk_utils);
        end

    end % methods
end % class arm_cf
