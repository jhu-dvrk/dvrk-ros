function sign_vel = sin_Vel(joint_vel, amplitude)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05

    if joint_vel >= abs(amplitude)
        sign_vel = 1;
    elseif joint_vel <= -abs(amplitude)
        sign_vel = 0;
    else
        sign_vel = 0.5+sin(pi*(joint_vel/amplitude)/2)/2;
    end
end