classdef mtm_gripper < dynamicprops
    properties (Access = protected)
        crtk_utils;
        ros_namespace;
    end

    methods

        function self = mtm_gripper(ros_namespace)
            self.ros_namespace = ros_namespace;
            self.crtk_utils = crtk.utils(self, ros_namespace);
            self.crtk_utils.add_measured_js();
        end

        function delete(self)
            delete(self.crtk_utils);
        end

    end % methods
end % class mtm_gripper
