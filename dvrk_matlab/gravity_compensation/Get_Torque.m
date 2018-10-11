function [effort] = Get_Torque(sub_tor)
%This is a copy of WPI script
%Link: https://github.com/WPI-AIM/dvrk_gravity_comp/blob/master/Final_Submission/MATLAB%20code/Regressor_and_Parameter_matrix/Get_Cof.m

%Use to get the torque values
    msg = receive(sub_tor);
    effort = msg.Effort;
end