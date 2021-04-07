classdef mtm_gripper < dynamicprops
    properties (Access = protected)
        crtk_utils;
        ros_namespace;
    end

    methods

        function self = mtm_gripper(name)
            self.ros_namespace = name;
            self.crtk_utils = crtk.utils(self, name);
            self.crtk_utils.add_measured_js();
        end

        function delete(self)
            delete(self.crtk_utils);
        end

    end

end
