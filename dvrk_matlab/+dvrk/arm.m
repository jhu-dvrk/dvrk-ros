classdef arm < dynamicprops
    % Class used to interface with ROS dVRK console topics and convert to useful
    % Matlab commands and properties.  To create a robot interface:
    %   r = arm('PSM1');
    %   r
    %
    % To home and check current state:
    %   r.home();
    %
    % Naming convention follows CRTK conventions

    % values set by this class, can be read by others
    properties (SetAccess = protected)
        crtk_utils;
        ros_namespace;
        body;
        spatial;
    end

    methods

        function [name] = name(self)
            name = self.ros_namespace;
        end

        function self = arm(name)
            self.ros_namespace = name;
            self.crtk_utils = crtk.utils(self, name);
            self.body = dvrk.arm_cf(strcat(name, '/body'));
            self.spatial = dvrk.arm_cf(strcat(name, '/spatial'));
            % operating state
	        self.crtk_utils.add_operating_state();
            % joint space
            self.crtk_utils.add_measured_js();
            self.crtk_utils.add_setpoint_js();
            self.crtk_utils.add_servo_jp();
            self.crtk_utils.add_servo_jf();
            self.crtk_utils.add_move_jp();
            % cartesian space
            self.crtk_utils.add_measured_cp();
            self.crtk_utils.add_measured_cv();
            self.crtk_utils.add_measured_cf();
            self.crtk_utils.add_setpoint_cp();
            self.crtk_utils.add_setpoint_cf();
            self.crtk_utils.add_servo_cp();
            self.crtk_utils.add_servo_cf();
            self.crtk_utils.add_move_cp();
        end

        function delete(self)
           delete(self.crtk_utils);
        end

    end % methods
end % class
