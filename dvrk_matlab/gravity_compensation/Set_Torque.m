function [Success] = Set_Torque(set_torque_msg,torque)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05

    msg = rosmessage(set_torque_msg);
    msg.Effort(1) = torque(1);
    msg.Effort(2) = torque(2);
    msg.Effort(3) = torque(3);
    msg.Effort(4) = torque(4);
    msg.Effort(5) = torque(5);
    msg.Effort(6) = torque(6);
    msg.Effort(7) = torque(7);
    send(set_torque_msg, msg);
    Success = 1;
end

