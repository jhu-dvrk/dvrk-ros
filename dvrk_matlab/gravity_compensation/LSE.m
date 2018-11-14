function [output_param_map,output_param_full_map, output_param_rel_std_map] = LSE(input_param_map,...
                                                                                    input_param_rel_std_map,...
                                                                                    R2_augmented,... 
                                                                                    T2_augmented,...
                                                                                    bool_Regressor_Mat,...
                                                                                    Train_Joint_No_list,...
                                                                                    Output_Param_Joint_No_list)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05
%  Reference: This is a modified version of WPI Code
%               link: https://github.com/WPI-AIM/dvrk_gravity_comp/blob/master/Final_Submission/MATLAB%20code/Regressor_and_Parameter_matrix/estimates_calulated.m
   
    param_num = size(R2_augmented,2);              
    %Set initial prior,  known_index(i)=1; known_parameter(i)={value}
    known_index = zeros(1,param_num);
    known_parameter = zeros(1,param_num);
    known_index(cell2mat(input_param_map.keys)) = 1;
    known_parameter(cell2mat(input_param_map.keys))=cell2mat(input_param_map.values);
    %Iterative Learning
    Trained_bool_Regressor_row = zeros(1,param_num);
    for i=1:size(Train_Joint_No_list,2)
        Trained_bool_Regressor_row = Trained_bool_Regressor_row | bool_Regressor_Mat(Train_Joint_No_list(i),:);
    end
    % simplify using known param and bool Regessor 
    T2 = T2_augmented;
    for i=1:param_num
        if (known_index(i) ==1)
            T2 = T2 - R2_augmented(:,i)*known_parameter(i);
        end
    end
    %get unknown index in bool mask 
    unknown_index = known_index~=1; 
    R2 = R2_augmented(:,unknown_index & Trained_bool_Regressor_row);


    %Compute unknown param using Least Square Estimation
    learn_dynamic_parameters = pinv(R2)*T2;

    %Compute the std of computed parameters
     [ std_var_beta, rel_std_var_beta ] = std_dynamic_param( R2, T2, learn_dynamic_parameters);

    %dynamic_param_result[:,1] = 1 for trained param; 2 for known param; 
    % 0 for param that this row cannot train
    k=1;
    dynamic_param_result = zeros(param_num,3);  
    for i = 1:param_num
        if known_index(i)==1
            dynamic_param_result(i,1:2) = [2,known_parameter(i)];
        elseif Trained_bool_Regressor_row(i)
            dynamic_param_result(i,:) = [1,learn_dynamic_parameters(k),rel_std_var_beta(k)];
            k = k+1;
        end
    end
    

    %Second column of dynamic_param_result: 
    %1 stand for trained param, 0 stands for unknown param, 2 stand for known param
    output_param_full_map = containers.Map(1:param_num,dynamic_param_result(:,2));
    output_param_map = containers.Map(1:param_num,dynamic_param_result(:,2));
    output_param_rel_std_map = containers.Map(1:param_num,dynamic_param_result(:,3));
    
    bool_joint_array =  zeros(1,param_num);
    for i=1:size(Output_Param_Joint_No_list,2)
        bool_joint_array = bool_joint_array | bool_Regressor_Mat(Output_Param_Joint_No_list(i),:);
    end
    
    
    for i=1:size(bool_joint_array, 2)
        if bool_joint_array(i)~=1
            output_param_map.remove(i);
            output_param_rel_std_map.remove(i);
        end
    end
    keys = input_param_rel_std_map.keys;
    values = input_param_rel_std_map.values;
    for i=1:size(input_param_rel_std_map.keys,2)
       output_param_rel_std_map(keys{i})  = values{i}; 
    end
end