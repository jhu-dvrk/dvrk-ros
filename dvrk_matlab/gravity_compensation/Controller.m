classdef Controller < handle
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05

    properties(Access = public)
        pub_tor
        sub_pos
        sub_pos_test
        dynamic_param
        dynamic_param_pos
        dynamic_param_neg
        safe_upper_torque_limit
        safe_lower_torque_limit
        beta_vel_amplitude
        g
        Zero_Output_Joint_No
        mtm_arm
        amplitude_vec
        msg_counter = 1000;
        msg_counter_buffer = 0
        move_joint_pause_time = 3
        is_start = false
        arm_name_string
    end
    
    methods(Access = public)
        function obj = Controller(pub_tor,...
                                  dynamic_param,...
                                  safe_upper_torque_limit,...
                                  safe_lower_torque_limit,...
                                  beta_vel_amplitude,...
                                  g,...
                                  Zero_Output_Joint_No,...
                                  mtm_arm,...
                                  amplitude_vec,...
                                  arm_name_string) % Constructor
                    obj.pub_tor = pub_tor;
                    obj.dynamic_param = dynamic_param;
                    obj.safe_upper_torque_limit = safe_upper_torque_limit;
                    obj.safe_lower_torque_limit = safe_lower_torque_limit;
                    obj.beta_vel_amplitude = beta_vel_amplitude; 
                    obj.g = g;
                    obj.Zero_Output_Joint_No = Zero_Output_Joint_No;
                    obj.mtm_arm = mtm_arm;
                    obj.amplitude_vec = amplitude_vec;
                    obj.arm_name_string = arm_name_string;
                    obj.dynamic_param_pos = dynamic_param([1:40]);
                    obj.dynamic_param_neg = dynamic_param([1:10,41:end]);
                    disp('Controller dynamic parameters of MTM: [param]');
                    for i=1:size(obj.dynamic_param_pos,1)
                        disp(sprintf('Param_%d: [%0.5f], [%0.5f] ', i, obj.dynamic_param_pos(i), obj.dynamic_param_neg(i)));
                    end
        end
        
        function assign_subcriber(obj, sub_pos)
            obj.sub_pos = sub_pos;
        end
        
        
        function callback_gc_publisher(obj, q, q_dot)
            if(~obj.is_start)
                disp(sprintf('GC of %s starts, you can move %s now. If you need to stop gc controller, call "rosshutdown".',obj.arm_name_string,obj.arm_name_string));
                obj.is_start = true;
            end
            
            if(obj.msg_counter_buffer==0)
                disp(sprintf('running..'));
            end                        
            
            if(obj.msg_counter==obj.msg_counter_buffer)
                obj.msg_counter_buffer = 0;
            else
                obj.msg_counter_buffer = obj.msg_counter_buffer+1;
            end
          Torques = obj.base_controller(q, q_dot);
          Set_Torque(obj.pub_tor, Torques);
        end
        
        function Torques = base_controller(obj, q, q_dot)
              qs = q;
              g = obj.g;
              q1 = qs(1);
              q2 = qs(2);
              q3 = qs(3);
              q4 = qs(4);
              q5 = qs(5);
              q6 = qs(6);
              q7 = qs(7);
              vel = q_dot;
              Regressor_Matrix_Pos =[         0,         0,                                     0,                                       0,                                                     0,                                                             0,                                                                                                                                      0,                                                                                                                                      0,                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                              0, 1, q1, q1^2, q1^3, q1^4, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
 g*sin(q2), g*cos(q2), g*cos(q2)*cos(q3) - g*sin(q2)*sin(q3), - g*cos(q2)*sin(q3) - g*cos(q3)*sin(q2), g*cos(q2)*cos(q3)*cos(q4) - g*cos(q4)*sin(q2)*sin(q3),         g*sin(q2)*sin(q3)*sin(q4) - g*cos(q2)*cos(q3)*sin(q4),          g*cos(q4)*sin(q2)*sin(q3)*sin(q5) - g*cos(q3)*cos(q5)*sin(q2) - g*cos(q2)*cos(q3)*cos(q4)*sin(q5) - g*cos(q2)*cos(q5)*sin(q3),          g*cos(q2)*cos(q3)*cos(q4)*cos(q5) - g*cos(q3)*sin(q2)*sin(q5) - g*cos(q2)*sin(q3)*sin(q5) - g*cos(q4)*cos(q5)*sin(q2)*sin(q3),         g*cos(q2)*cos(q3)*sin(q4)*sin(q6) + g*cos(q2)*cos(q6)*sin(q3)*sin(q5) + g*cos(q3)*cos(q6)*sin(q2)*sin(q5) - g*sin(q2)*sin(q3)*sin(q4)*sin(q6) + g*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) - g*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6),          g*cos(q2)*cos(q3)*cos(q6)*sin(q4) - g*cos(q6)*sin(q2)*sin(q3)*sin(q4) - g*cos(q2)*sin(q3)*sin(q5)*sin(q6) - g*cos(q3)*sin(q2)*sin(q5)*sin(q6) - g*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6) + g*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6), 0,  0,    0,    0,    0, 1, q2, q2^2, q2^3, q2^4, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
         0,         0,                        g*cos(q2 + q3),                         -g*sin(q2 + q3),         (g*(cos(q2 + q3 + q4) + cos(q2 + q3 - q4)))/2, (g*(2*sin(q2)*sin(q3)*sin(q4) - 2*cos(q2)*cos(q3)*sin(q4)))/2, -(g*(2*cos(q2)*cos(q5)*sin(q3) + 2*cos(q3)*cos(q5)*sin(q2) + 2*cos(q2)*cos(q3)*cos(q4)*sin(q5) - 2*cos(q4)*sin(q2)*sin(q3)*sin(q5)))/2, -(g*(2*cos(q2)*sin(q3)*sin(q5) + 2*cos(q3)*sin(q2)*sin(q5) - 2*cos(q2)*cos(q3)*cos(q4)*cos(q5) + 2*cos(q4)*cos(q5)*sin(q2)*sin(q3)))/2, (g*(2*cos(q2)*cos(q3)*sin(q4)*sin(q6) + 2*cos(q2)*cos(q6)*sin(q3)*sin(q5) + 2*cos(q3)*cos(q6)*sin(q2)*sin(q5) - 2*sin(q2)*sin(q3)*sin(q4)*sin(q6) - 2*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) + 2*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3)))/2, -(g*(2*cos(q6)*sin(q2)*sin(q3)*sin(q4) - 2*cos(q2)*cos(q3)*cos(q6)*sin(q4) + 2*cos(q2)*sin(q3)*sin(q5)*sin(q6) + 2*cos(q3)*sin(q2)*sin(q5)*sin(q6) - 2*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) + 2*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6)))/2, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 1, q3, q3^2, q3^3, q3^4, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
         0,         0,                                     0,                                       0,                               -g*sin(q2 + q3)*sin(q4),                                       -g*sin(q2 + q3)*cos(q4),                                                                                                         g*sin(q2 + q3)*sin(q4)*sin(q5),                                                                                                        -g*sin(q2 + q3)*cos(q5)*sin(q4),                                                                                                                                                                                    g*sin(q2 + q3)*(cos(q4)*sin(q6) + cos(q5)*cos(q6)*sin(q4)),                                                                                                                                                                                     g*sin(q2 + q3)*(cos(q4)*cos(q6) - cos(q5)*sin(q4)*sin(q6)), 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 1, q4, q4^2, q4^3, q4^4, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
         0,         0,                                     0,                                       0,                                                     0,                                                             0,             -g*(cos(q2)*cos(q3)*sin(q5) - sin(q2)*sin(q3)*sin(q5) + cos(q2)*cos(q4)*cos(q5)*sin(q3) + cos(q3)*cos(q4)*cos(q5)*sin(q2)),             -g*(cos(q5)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*cos(q5) + cos(q2)*cos(q4)*sin(q3)*sin(q5) + cos(q3)*cos(q4)*sin(q2)*sin(q5)),                                                                                     g*(cos(q5)*cos(q6)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*cos(q5)*cos(q6) + cos(q2)*cos(q4)*cos(q6)*sin(q3)*sin(q5) + cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q5)),                                                                                     -g*(cos(q5)*sin(q2)*sin(q3)*sin(q6) - cos(q2)*cos(q3)*cos(q5)*sin(q6) + cos(q2)*cos(q4)*sin(q3)*sin(q5)*sin(q6) + cos(q3)*cos(q4)*sin(q2)*sin(q5)*sin(q6)), 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 1, q5, q5^2, q5^3, q5^4, 0,  0,    0,    0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
         0,         0,                                     0,                                       0,                                                     0,                                                             0,                                                                                                                                      0,                                                                                                                                      0,                 g*(cos(q2)*cos(q6)*sin(q3)*sin(q4) + cos(q3)*cos(q6)*sin(q2)*sin(q4) + cos(q2)*cos(q3)*sin(q5)*sin(q6) - sin(q2)*sin(q3)*sin(q5)*sin(q6) + cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) + cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6)),                  g*(cos(q2)*cos(q3)*cos(q6)*sin(q5) - cos(q2)*sin(q3)*sin(q4)*sin(q6) - cos(q3)*sin(q2)*sin(q4)*sin(q6) - cos(q6)*sin(q2)*sin(q3)*sin(q5) + cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) + cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2)), 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 1, q6, q6^2, q6^3, q6^4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
         0,         0,                                     0,                                       0,                                                     0,                                                             0,                                                                                                                                      0,                                                                                                                                      0,                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                              0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
 
        Regressor_Matrix_Neg = [         0,         0,                                     0,                                       0,                                                     0,                                                             0,                                                                                                                                      0,                                                                                                                                      0,                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, q1, q1^2, q1^3, q1^4, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0
 g*sin(q2), g*cos(q2), g*cos(q2)*cos(q3) - g*sin(q2)*sin(q3), - g*cos(q2)*sin(q3) - g*cos(q3)*sin(q2), g*cos(q2)*cos(q3)*cos(q4) - g*cos(q4)*sin(q2)*sin(q3),         g*sin(q2)*sin(q3)*sin(q4) - g*cos(q2)*cos(q3)*sin(q4),          g*cos(q4)*sin(q2)*sin(q3)*sin(q5) - g*cos(q3)*cos(q5)*sin(q2) - g*cos(q2)*cos(q3)*cos(q4)*sin(q5) - g*cos(q2)*cos(q5)*sin(q3),          g*cos(q2)*cos(q3)*cos(q4)*cos(q5) - g*cos(q3)*sin(q2)*sin(q5) - g*cos(q2)*sin(q3)*sin(q5) - g*cos(q4)*cos(q5)*sin(q2)*sin(q3),         g*cos(q2)*cos(q3)*sin(q4)*sin(q6) + g*cos(q2)*cos(q6)*sin(q3)*sin(q5) + g*cos(q3)*cos(q6)*sin(q2)*sin(q5) - g*sin(q2)*sin(q3)*sin(q4)*sin(q6) + g*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) - g*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6),          g*cos(q2)*cos(q3)*cos(q6)*sin(q4) - g*cos(q6)*sin(q2)*sin(q3)*sin(q4) - g*cos(q2)*sin(q3)*sin(q5)*sin(q6) - g*cos(q3)*sin(q2)*sin(q5)*sin(q6) - g*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6) + g*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,    0,    0,    0, 1, q2, q2^2, q2^3, q2^4, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0
         0,         0,                        g*cos(q2 + q3),                         -g*sin(q2 + q3),         (g*(cos(q2 + q3 + q4) + cos(q2 + q3 - q4)))/2, (g*(2*sin(q2)*sin(q3)*sin(q4) - 2*cos(q2)*cos(q3)*sin(q4)))/2, -(g*(2*cos(q2)*cos(q5)*sin(q3) + 2*cos(q3)*cos(q5)*sin(q2) + 2*cos(q2)*cos(q3)*cos(q4)*sin(q5) - 2*cos(q4)*sin(q2)*sin(q3)*sin(q5)))/2, -(g*(2*cos(q2)*sin(q3)*sin(q5) + 2*cos(q3)*sin(q2)*sin(q5) - 2*cos(q2)*cos(q3)*cos(q4)*cos(q5) + 2*cos(q4)*cos(q5)*sin(q2)*sin(q3)))/2, (g*(2*cos(q2)*cos(q3)*sin(q4)*sin(q6) + 2*cos(q2)*cos(q6)*sin(q3)*sin(q5) + 2*cos(q3)*cos(q6)*sin(q2)*sin(q5) - 2*sin(q2)*sin(q3)*sin(q4)*sin(q6) - 2*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) + 2*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3)))/2, -(g*(2*cos(q6)*sin(q2)*sin(q3)*sin(q4) - 2*cos(q2)*cos(q3)*cos(q6)*sin(q4) + 2*cos(q2)*sin(q3)*sin(q5)*sin(q6) + 2*cos(q3)*sin(q2)*sin(q5)*sin(q6) - 2*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) + 2*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6)))/2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 1, q3, q3^2, q3^3, q3^4, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0
         0,         0,                                     0,                                       0,                               -g*sin(q2 + q3)*sin(q4),                                       -g*sin(q2 + q3)*cos(q4),                                                                                                         g*sin(q2 + q3)*sin(q4)*sin(q5),                                                                                                        -g*sin(q2 + q3)*cos(q5)*sin(q4),                                                                                                                                                                                    g*sin(q2 + q3)*(cos(q4)*sin(q6) + cos(q5)*cos(q6)*sin(q4)),                                                                                                                                                                                     g*sin(q2 + q3)*(cos(q4)*cos(q6) - cos(q5)*sin(q4)*sin(q6)), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 1, q4, q4^2, q4^3, q4^4, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0
         0,         0,                                     0,                                       0,                                                     0,                                                             0,             -g*(cos(q2)*cos(q3)*sin(q5) - sin(q2)*sin(q3)*sin(q5) + cos(q2)*cos(q4)*cos(q5)*sin(q3) + cos(q3)*cos(q4)*cos(q5)*sin(q2)),             -g*(cos(q5)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*cos(q5) + cos(q2)*cos(q4)*sin(q3)*sin(q5) + cos(q3)*cos(q4)*sin(q2)*sin(q5)),                                                                                     g*(cos(q5)*cos(q6)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*cos(q5)*cos(q6) + cos(q2)*cos(q4)*cos(q6)*sin(q3)*sin(q5) + cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q5)),                                                                                     -g*(cos(q5)*sin(q2)*sin(q3)*sin(q6) - cos(q2)*cos(q3)*cos(q5)*sin(q6) + cos(q2)*cos(q4)*sin(q3)*sin(q5)*sin(q6) + cos(q3)*cos(q4)*sin(q2)*sin(q5)*sin(q6)), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 1, q5, q5^2, q5^3, q5^4, 0,  0,    0,    0,    0
         0,         0,                                     0,                                       0,                                                     0,                                                             0,                                                                                                                                      0,                                                                                                                                      0,                 g*(cos(q2)*cos(q6)*sin(q3)*sin(q4) + cos(q3)*cos(q6)*sin(q2)*sin(q4) + cos(q2)*cos(q3)*sin(q5)*sin(q6) - sin(q2)*sin(q3)*sin(q5)*sin(q6) + cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) + cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6)),                  g*(cos(q2)*cos(q3)*cos(q6)*sin(q5) - cos(q2)*sin(q3)*sin(q4)*sin(q6) - cos(q3)*sin(q2)*sin(q4)*sin(q6) - cos(q6)*sin(q2)*sin(q3)*sin(q5) + cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) + cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2)), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 1, q6, q6^2, q6^3, q6^4
         0,         0,                                     0,                                       0,                                                     0,                                                             0,                                                                                                                                      0,                                                                                                                                      0,                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0];
 
 
        
          Torques_pos = Regressor_Matrix_Pos*obj.dynamic_param;
          Torques_neg = Regressor_Matrix_Neg*obj.dynamic_param;
          Torques = zeros(7,1);
          for i =1:6
              % Blend torques with positive and negative direction into output torques
              alpha = sin_Vel(vel(i),obj.beta_vel_amplitude(i));
              Torques(i) = Torques_pos(i)*alpha+Torques_neg(i)*(1-alpha);
              
              % Set upper and lower torque limit
              if Torques(i)>=obj.safe_upper_torque_limit(i)
                   Torques(i)=obj.safe_upper_torque_limit(i);
              elseif Torques(i)<=obj.safe_lower_torque_limit(i)
                    Torques(i)=obj.safe_lower_torque_limit(i); 
              end
          end
          Torques(obj.Zero_Output_Joint_No) = 0;
        end
        
        function [abs_err, rel_err] = controller_test(obj, q)
            obj.mtm_arm.move_joint(q);
            pause(obj.move_joint_pause_time);
            q = Get_Position(obj.sub_pos);
            q_dot = Get_Velocity(obj.sub_pos);
            tau_measured = Get_Torque(obj.sub_pos);
            tau_computed = obj.base_controller(q,q_dot);
            abs_err = abs(tau_measured-tau_computed);
            rel_err = abs(abs_err./obj.amplitude_vec);
        end
    end
    
end

