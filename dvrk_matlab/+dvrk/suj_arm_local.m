classdef suj_arm_local < dynamicprops
    properties (Access = protected)
        crtk_utils;
        ros_namespace;
    end

    methods

        function self = suj_arm_local(ros_namespace)
            self.ros_namespace = ros_namespace;
            self.crtk_utils = crtk.utils(self, ros_namespace);
            self.crtk_utils.add_measured_cp();
        end

        function delete(self)
            delete(self.crtk_utils);
        end

    end

end