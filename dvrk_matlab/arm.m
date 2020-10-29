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
    properties (Access = private)
        crtk_utils;
    end

    methods

        function self = arm(ros_namespace)
            self.crtk_utils = crtk_utils(self, ros_namespace);
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
            self.crtk_utils.add_body_measured_cf();
            self.crtk_utils.add_setpoint_cp();
            self.crtk_utils.add_setpoint_cf();
            self.crtk_utils.add_servo_cp();
            self.crtk_utils.add_spatial_servo_cf();
            self.crtk_utils.add_move_cp();
        end

        function delete(self)
           delete(self.crtk_utils);
        end

    end % methods
end % class
