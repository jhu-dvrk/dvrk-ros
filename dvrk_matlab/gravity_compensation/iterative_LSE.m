function [output_param_map,output_param_full_map, output_param_rel_std_map] = iterative_LSE(input_param_map,...
                                                                                            input_param_rel_std_map,...
                                                                                            R2_augmented,... 
                                                                                            T2_augmented,...
                                                                                            bool_Regressor_Mat,...
                                                                                            Output_Param_Joint_No)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05
%  Reference: This is a modified version of WPI Code
%               link: https://github.com/WPI-AIM/dvrk_gravity_comp/blob/master/Final_Submission/MATLAB%20code/Regressor_and_Parameter_matrix/estimates_calulated.m
   
    size_R = size(R2_augmented); size_T = size(T2_augmented); 
     size_bool_R = size(bool_Regressor_Mat);
    if size_R(1)~=size_T(1)  | size_bool_R ~= size_R
        error('Please gives the correct size of argument')
    else
    param_num = size_R(2);              
    %Set initial prior,  known_index(i)=1; known_parameter(i)={value}
    known_index = zeros(param_num,1);
    known_parameter = zeros(param_num,1);
    known_index(cell2mat(input_param_map.keys)) = 1;
    known_parameter(cell2mat(input_param_map.keys))=cell2mat(input_param_map.values);
    %Iterative Learning
    for j = 7:-1:min(Output_Param_Joint_No)
        % simplify using known param and bool Regessor 
        T2 = T2_augmented;
        for i=1:param_num
            if (known_index(i) ==1)
                T2 = T2 - R2_augmented(:,i)*known_parameter(i);
            end
        end
        %get unknown index in bool mask 
        unknown_index = known_index~=1; 
        unknown_index= unknown_index.';
        bool_unknown_index = unknown_index & bool_Regressor_Mat(j,:); 
        R2 = R2_augmented(:,bool_unknown_index);

        % Extracting the rows from T2 and R2 which to be trained 
        Train_Row_Index = zeros(1,7);Train_Row_Index(j) = 1;
        Train_Row_list = [];
        d_size = size(T2);
        for i = 1:(d_size(1)/7)
            Train_Row_list = [Train_Row_list,Train_Row_Index];
        end
        bool_Train_Row = Train_Row_list == 1;
        R2 = R2(bool_Train_Row,:);
        T2 = T2(bool_Train_Row,:);

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
            elseif bool_Regressor_Mat(j,i)
                dynamic_param_result(i,:) = [1,learn_dynamic_parameters(k),rel_std_var_beta(k)];
                k = k+1;
            end
        end

        % Set trained param to known param for the next round training
        for i = 1:param_num
            if(dynamic_param_result(i,1)==1)
                known_index(i)=1;
                known_parameter(i)=dynamic_param_result(i,2);
            end
        end
    end

    %Second column of dynamic_param_result: 
    %1 stand for trained param, 0 stands for unknown param, 2 stand for known param
    output_param_full_map = containers.Map(1:param_num,dynamic_param_result(:,2));
    output_param_map = containers.Map(1:param_num,dynamic_param_result(:,2));
    output_param_rel_std_map = containers.Map(1:param_num,dynamic_param_result(:,3));
    
    bool_joint_array =  zeros(1,param_num);
    for i=1:size(Output_Param_Joint_No,2)
        bool_joint_array = bool_joint_array | bool_Regressor_Mat(Output_Param_Joint_No(i),:);
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
    output_param_rel_std_map
end