classdef psm_jaw < dynamicprops
    properties (Access = protected)
        crtk_utils;
        ros_namespace;
    end

    methods

        function self = psm_jaw(ros_namespace)
            self.ros_namespace = ros_namespace;
            self.crtk_utils = crtk.utils(self, ros_namespace);
            self.crtk_utils.add_measured_js();
            self.crtk_utils.add_setpoint_js();
            self.crtk_utils.add_servo_jp();
            self.crtk_utils.add_servo_jf();
            self.crtk_utils.add_move_jp();
        end

        function delete(self)
            delete(self.crtk_utils);
        end

    end % methods
end % class psm_jaw
