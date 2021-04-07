classdef suj_arm < dynamicprops
    properties (Access = protected)
        crtk_utils;
        ros_namespace;
    end

    properties (SetAccess = immutable)
        local;
    end

    methods

        function self = suj_arm(name)
            self.ros_namespace = name;
            self.crtk_utils = crtk.utils(self, name);
            self.crtk_utils.add_operating_state();
            self.crtk_utils.add_measured_js();
            self.crtk_utils.add_measured_cp();
            self.crtk_utils.add_servo_jp();
            % local
            self.local = dvrk.suj_arm_local(strcat(name, '/local'));
        end

        function delete(self)
            delete(self.crtk_utils);
            delete(self.local);
        end

    end

end
