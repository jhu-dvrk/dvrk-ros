classdef arm_cf < dynamicprops
    properties (Access = protected)
        crtk_utils;
        ros_namespace;
    end

    methods

        function self = arm_cf(name)
            self.ros_namespace = name;
            self.crtk_utils = crtk.utils(self, name);
            self.crtk_utils.add_measured_cf();
            self.crtk_utils.add_servo_cf();
        end

        function delete(self)
            delete(self.crtk_utils);
        end

    end

end
