function [is_lse_success, dynamic_parameters] = friction_Model(Joint_No,...
    std_filter,...
    g_constant,...
    Is_Plot,...
    issave_figure,...
    Input_pos_Data_Path,...
    input_pos_data_files,...
    Input_neg_Data_Path,...
    input_neg_data_files,...
    new_param_save_path,...
    new_fig_save_path,...
    prior_param_index,...
    prior_param_values,...
    Output_Param_Joint_No,...
    old_param_path)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05
%  Reference: This is a modified version of WPI Code
%               link: https://github.com/WPI-AIM/dvrk_gravity_comp/blob/master/Final_Submission/MATLAB%20code/Regressor_and_Parameter_matrix/plug_data.m


    % Get Data and Data Processing
    if exist(Input_pos_Data_Path)==0
       is_lse_success = false;
       error(sprintf('Cannot find input data folder: %s, Please check if input data folder exist. ',Input_Data_Path));
       dynamic_parameters = 0;
       return
       %error(['input path dose not exist! ', Input_Data_Path])
    end
    data_path_struct_list = dir(input_pos_data_files);
    Torques_pos_data_list = {};
    theta_pos_list = {};
    for i=1:size(data_path_struct_list,1)
        load(strcat(data_path_struct_list(i).folder,'/',data_path_struct_list(i).name));
        Torques_data = Torques_data_Process(current_position, desired_effort, 'mean',std_filter);
        Torques_pos_data_list{end+1} = Torques_data;
        theta_pos_list{end+1} = int32(Theta);
    end
    
    if exist(Input_neg_Data_Path)==0
       is_lse_success = false;
       error(sprintf('Cannot find input data folder: %s, Please check if input data folder exist. ',Input_Data_Path));
       dynamic_parameters = 0;
       return
       %error(['input path dose not exist! ', Input_Data_Path])
    end
    data_path_struct_list = dir(input_neg_data_files);
    Torques_neg_data_list = {};
    theta_neg_list = {};
    for i=1:size(data_path_struct_list,1)
        load(strcat(data_path_struct_list(i).folder,'/',data_path_struct_list(i).name));
        Torques_data = Torques_data_Process(current_position, desired_effort, 'mean',std_filter);
        Torques_neg_data_list{end+1} = Torques_data;
        theta_neg_list{end+1} = int32(Theta);
    end


    % Append List Torques Data
    Torques_data = [];
    for j = 1:size(Torques_data_list,2)
        Torques_data = cat(3,Torques_data,Torques_data_list{j});
    end

    % Plug Torques Data to Augumented Regressor Matrix and Torque Matrix
    [Regressor_Matrix, Parameter_matrix,bool_Regressor_Mat] = symbolic_gc_dynamic();
    %disp(sprintf('Data transformation into augumented matrix....',Joint_No));
    [R2_augmented, T2_augmented] = data2augMat(Torques_data, Regressor_Matrix, g_constant);


    %% GC Parameters Estimation and Evaluation

    if exist('old_param_path')
        load(strcat(old_param_path,'/param.mat'));
        prior_param_map = containers.Map(prior_param_index,prior_param_values);
        prior_param_map.remove(0);

        old_param_index = output_param_map.keys;
        old_param_value = output_param_map.values;
        old_param_map = output_param_map;

        %Prevent overlap param between prior and old values
        prior_k = prior_param_map.keys;
        old_index_array = cell2mat(old_param_index);
        if size(prior_k,2)~=0
            for j=1:size(prior_k,2)
                if ismember(prior_k{j}, old_index_array)
                    prior_param_map.remove(prior_k{j});
                end
            end
        end
        prior_param_index = prior_param_map.keys;
        prior_param_values = prior_param_map.values;

        %Merge old and prior values
        input_param_index = horzcat(old_param_index, prior_param_index);
        input_param_value = horzcat(old_param_value, prior_param_values);
        input_param_map = containers.Map(input_param_index, input_param_value);
    else
        input_param_map = containers.Map(prior_param_index, prior_param_values);
        input_param_map.remove(0);
    end

    [output_param_map,output_param_full_map] = iterative_LSE(input_param_map, ...
                                         R2_augmented, T2_augmented,bool_Regressor_Mat, Output_Param_Joint_No);

    output_keys = output_param_map.keys;
    output_values = output_param_map.values;
    dynamic_parameters = output_param_full_map.values.';
    
    disp('Output Parameters:');
    for i=1:size(output_param_map.keys,2)
        disp(sprintf('Param_%d: %0.4f',output_keys{i}, output_values{i}));
    end

    % Plot the Predict line and real torque data
    if Is_Plot~=0 | issave_figure~=0
        dynamic_parameters = cell2mat(output_param_full_map.values);
        dynamic_parameters = dynamic_parameters.';
        for j = 1:size(Torques_data_list,2)
            if(size(Torques_data_list{j},3)~=0  )
            plot_Fit_Joint(Torques_data_list{j},Regressor_Matrix, dynamic_parameters,...
                Joint_No, theta_list{j},Is_Plot, issave_figure, new_fig_save_path);
            end
        end
    end

    % Save param
    if exist(new_param_save_path)~=7
        mkdir(new_param_save_path)
    end
    save(strcat(new_param_save_path,'/param.mat'),'output_param_map','output_param_full_map');
    is_lse_success = true;
end