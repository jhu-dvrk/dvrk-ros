function dynamic_parameters_vec = LSE_Model(config_lse_joint1,...
                                        old_param_path)
                                                       
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05
%  Reference: This is a modified version of WPI Code
%               link: https://github.com/WPI-AIM/dvrk_gravity_comp/blob/master/Final_Submission/MATLAB%20code/Regressor_and_Parameter_matrix/plug_data.m
       

    % Plug Torques Data to Augumented Regressor Matrix and Torque Matrix
    [Regressor_Matrix_Pos,Regressor_Matrix_Neg, Parameter_matrix, bool_Regressor_Mat] = symbolic_gc_dynamic();    
    
    lse_obj1 = lse_preparation(config_lse_joint1,Regressor_Matrix_Pos,Regressor_Matrix_Neg);

    R2_augmented = [lse_obj1.R2_augmented_pos; lse_obj1.R2_augmented_neg];
    T2_augmented = [lse_obj1.T2_augmented_pos; lse_obj1.T2_augmented_neg];
    
    % Generate the input argument,input_param_map and input_param_rel_std_map, for LSE
    if exist('old_param_path')
        % If there is old parameters which is trained by the previous step, we need to simplify the regressor and torque matrix using the old paramters
        load(strcat(old_param_path,'/param.mat'));
        old_param_map = output_param_map;
        old_param_rel_std_map = output_param_rel_std_map;

        % If prior overlaps with old param values, we choose use old param values instead of piror values.
        prior_param_map = containers.Map(lse_obj1.prior_param_index,lse_obj1.prior_param_values);
        prior_param_map.remove(0);
        prior_k = prior_param_map.keys;
        old_index_array = cell2mat(output_param_map.keys);
        if size(prior_k,2)~=0
            for j=1:size(prior_k,2)
                if ismember(prior_k{j}, old_index_array)
                    prior_param_map.remove(prior_k{j});
                end
            end
        end
        prior_param_index = prior_param_map.keys;
        prior_param_values = prior_param_map.values;

        % input_param_map =[old_param_map,prior_param_map]
        input_param_index = horzcat(old_param_map.keys, prior_param_index);
        input_param_value = horzcat(old_param_map.values, prior_param_values);
        input_param_map = containers.Map(input_param_index, input_param_value);
        
        %input_param_rel_std_map = old_param_rel_std_map
        input_param_rel_std_map = containers.Map(old_param_rel_std_map.keys,old_param_rel_std_map.values);
    else
        % input_param_map = empty
        input_param_map = containers.Map(config_lse_joint1.prior_param_index, config_lse_joint1.prior_param_values);
        input_param_map.remove(0);
        
        % input_param_rel_std_map = empty
        input_param_rel_std_map = containers.Map({0},{0});
        input_param_rel_std_map.remove(0);
    end
    
    % general config
    Output_Param_Joint_No = lse_obj1.Output_Param_Joint_No;
    Train_Joint_No = lse_obj1.Joint_No;

    [output_param_map,output_param_full_map, output_param_rel_std_map] = LSE(input_param_map,...
                                                                            input_param_rel_std_map,...
                                                                            R2_augmented,... 
                                                                            T2_augmented,...
                                                                            bool_Regressor_Mat,...
                                                                            Train_Joint_No,...
                                                                            Output_Param_Joint_No);

    output_keys = output_param_map.keys;
    output_values = output_param_map.values;
    dynamic_parameters_vec = zeros(size(output_param_full_map.values,2),1);
    dynamic_parameters_vec(:) = cell2mat(output_param_full_map.values.');
    output_param_rel_std_values = output_param_rel_std_map.values;
    
    disp('Output Parameters: [Value], [Relative_std(%)]');
    for i=1:size(output_param_map.keys,2)
        disp(sprintf('Param_%d: [%0.4f], [%0.1f%%]',output_keys{i}, output_values{i}, output_param_rel_std_values{i}));
    end
    disp(' ');
    
    % Plot the fitting figures using trained parameters
    if lse_obj1.Is_Plot~=0 | lse_obj1.issave_figure~=0
        plot_fitting_curves(lse_obj1.Torques_pos_data_list,lse_obj1.theta_pos_list,lse_obj1.Regressor_Matrix_Pos,dynamic_param_vec);
        plot_fitting_curves(lse_obj1.Torques_neg_data_list,lse_obj2.theta_neg_list,lse_obj1.Regressor_Matrix_Neg,dynamic_param_vec);
    end

    % Save param to file
    if exist(lse_obj1.new_param_save_path)~=7
        mkdir(lse_obj1.new_param_save_path)
    end
    save(strcat(lse_obj1.new_param_save_path,'/param.mat'),'output_param_map','output_param_full_map','output_param_rel_std_map');
   
    % Sucess flag
    is_lse_success = true;
end

%%
function lse_obj = lse_preparation(config_lse_joint,Regressor_Matrix_Pos,Regressor_Matrix_Neg)

    % create lse_obj inheriting config_lse_joint
    lse_obj.Joint_No = config_lse_joint.Joint_No;
    lse_obj.std_filter = config_lse_joint.std_filter;
    lse_obj.g_constant = config_lse_joint.g_constant;
    lse_obj.Is_Plot = config_lse_joint.Is_Plot;
    lse_obj.issave_figure = config_lse_joint.issave_figure;
    lse_obj.Input_Pos_Data_Path = config_lse_joint.input_pos_data_path; 
    lse_obj.Input_Neg_Data_Path = config_lse_joint.input_neg_data_path; 
    lse_obj.input_pos_data_files = config_lse_joint.input_pos_data_files;
    lse_obj.input_neg_data_files = config_lse_joint.input_neg_data_files;
    lse_obj.new_param_save_path = config_lse_joint.output_param_path;
    lse_obj.new_fig_pos_save_path = config_lse_joint.output_pos_fig_path;
    lse_obj.new_fig_neg_save_path = config_lse_joint.output_neg_fig_path;
    lse_obj.prior_param_index = config_lse_joint.prior_param_index;
    lse_obj.prior_param_values = config_lse_joint.prior_param_values;
    lse_obj.Output_Param_Joint_No = config_lse_joint.Output_Param_Joint_No;
    lse_obj.std_filter = config_lse_joint.std_filter;

    % check the given joint path exist
    if exist(lse_obj.Input_Pos_Data_Path)==0
       error(sprintf('Cannot find input data folder: %s, Please check if input data folder exist. ',lse_obj.Input_Pos_Data_Path));
    end
    if exist(lse_obj.Input_Neg_Data_Path)==0
       error(sprintf('Cannot find input data folder: %s, Please check if input data folder exist. ',lse_obj.Input_Neg_Data_Path));
    end
    
    data_path_struct_list = dir(lse_obj.input_pos_data_files);
    lse_obj.Torques_pos_data_list = {};
    lse_obj.theta_pos_list = {};
    for i=1:size(data_path_struct_list,1)
        load(strcat(data_path_struct_list(i).folder,'/',data_path_struct_list(i).name));
        lse_obj.Torques_pos_data_list{end+1} = Torques_data_Process(current_position,...
                                                                    desired_effort,...
                                                                    'mean',...
                                                                    lse_obj.std_filter);
        lse_obj.theta_pos_list{end+1} = int32(Theta);
    end
    
    data_path_struct_list = dir(lse_obj.input_neg_data_files);
    lse_obj.Torques_neg_data_list = {};
    lse_obj.theta_neg_list = {};
    for i=1:size(data_path_struct_list,1)
        load(strcat(data_path_struct_list(i).folder,'/',data_path_struct_list(i).name));
         lse_obj.Torques_neg_data_list{end+1} = Torques_data_Process(current_position,...
                                                                    desired_effort,...
                                                                    'mean',...
                                                                    lse_obj.std_filter);
        lse_obj.theta_neg_list{end+1} = int32(Theta);
    end
    
     % Append List Torques Data
    lse_obj.Torques_pos_data = [];
    for j = 1:size(lse_obj.Torques_pos_data_list,2)
        lse_obj.Torques_pos_data = cat(3,lse_obj.Torques_pos_data,lse_obj.Torques_pos_data_list{j});
    end
    lse_obj.Torques_neg_data = [];
    for j = 1:size(lse_obj.Torques_neg_data_list,2)
        lse_obj.Torques_neg_data = cat(3,lse_obj.Torques_neg_data,lse_obj.Torques_neg_data_list{j});
    end
    
    [lse_obj.R2_augmented_pos, lse_obj.T2_augmented_pos] = data2augMat(lse_obj.Torques_pos_data,...
                                                       lse_obj.Joint_No,...
                                                       Regressor_Matrix_Pos,...
                                                       lse_obj.g_constant);
    
    [lse_obj.R2_augmented_neg, lse_obj.T2_augmented_neg] = data2augMat(lse_obj.Torques_neg_data,...
                                                       lse_obj.Joint_No,...
                                                       Regressor_Matrix_Neg,...
                                                       lse_obj.g_constant);
    
end
%%
function plot_fitting_curves(Torques_data_list,theta_list,Regressor_Matrix,dynamic_param_vec)
    for j = 1:size(Torques_data_list,2)
        if(size(Torques_data_list{j},3)~=0  )
        plot_Fit_Joint(Torques_data_list{j},Regressor_Matrix, dynamic_param_vec,...
            Joint_No, theta_list{j},g_constant,Is_Plot, issave_figure, new_fig_pos_save_path, j);
        end
    end
end