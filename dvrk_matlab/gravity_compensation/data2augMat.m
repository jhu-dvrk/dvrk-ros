function  [R2_augmented, T2_augmented] = data2augMat(Torques_data,...
                                                     Joint_No,...
                                                     sym_R_mat,...
                                                     g)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05

    %% Plug Training Data to Augumented Regressor Matrix and Augumented Torque Matrix
    % number of data sets
    param_num = size(sym_R_mat,2);
    
    R2_augmented = [];
    T2_augmented = [];

    q_sub = zeros(7,1); 
    tor_sub = zeros(7,1);
    syms_R_row = sym_R_mat(Joint_No,:);
    symvar_row = symvar(sym_R_mat(Joint_No,:));
    for i=1:size(Torques_data,3)
        var_sym2value_map = containers.Map({'q1','q2','q3','q4','q5','q6','q7','g'},...
                                            num2cell([Torques_data(1:7,1,i).',g]));
        var_list = [];
        for k=1:size(symvar_row,2)
            var_list(end+1) = var_sym2value_map(char(symvar_row(k)));
        end
        A = double(subs(syms_R_row,symvar_row,var_list));
        tor_sub = Torques_data(Joint_No,2,i);
       
        R2_augmented = [R2_augmented;A];
        T2_augmented = [T2_augmented;tor_sub];
    end 

end