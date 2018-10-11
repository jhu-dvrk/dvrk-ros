function GC_Controllers = GC_controller(GC_ARM,...
                                        GC_controller_config_json_str,...
                                        dynamic_paramters_root_path_or_matrix,...
                                        fit_method,...
                                        g_constant)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05

                    
    % Define paramters number of GC
    dynamic_param_num = 41; % hardcode constant, do not change. 
        
    % Argument Checking
    if~(strcmp(GC_ARM,'MTML') | strcmp(GC_ARM,'MTMR') | strcmp(GC_ARM,'MTML&MTMR'))
        error(sprintf(['Input of argument ''GC_ARM''= %s is error, you should input',...
                       'one of the string[''MTML'',''MTMR'',''MTML&MTMR'']'],GC_ARM));
    end
    if~(strcmp(GC_controller_config_json_str(end-4:end),'.json'))
        error(sprintf(['Input of argument ''GC_controller_config_json_str''= %s',...
                      ' is error, you should input file with .json extend format '],GC_controller_config_json_str));
    end
    if ischar(dynamic_paramters_root_path_or_matrix)
        if exist(dynamic_paramters_root_path_or_matrix)~=7
             error(sprintf(['Input of argument ''dynamic_paramters_root_path_or_matrix''=',...
                             ' ''%s'' do not exist, please check your input path'],dynamic_paramters_root_path_or_matrix));
        end
    elseif ismatrix(dynamic_paramters_root_path_or_matrix) & size(dynamic_paramters_root_path_or_matrix,2)>1 
        if size(dynamic_paramters_root_path_or_matrix,1) ~= dynamic_param_num
            error(sprintf(['Input of argument ''dynamic_paramters_root_path_or_matrix'' has wrong column number,',...
                          ' it should be %d instead of %d'],dynamic_param_num,size(dynamic_paramters_root_path_or_matrix,1)));
        end
    else
        error(sprintf('Input of argument ''dynamic_paramter'' == %s is neither path string or matrix',...
                       dynamic_paramters_root_path_or_matrix));
    end
    if~(ismember(fit_method,{'original', 'drift', '1POL', '2POL', '3POL', '4POL'}))
        error(sprintf(['Input of argument ''fit_method''= ''%s'' is error, you should input one of the string',...
                      ' {''original'', ''drift'', ''1POL'', ''2POL'', ''3POL'', ''4POL''}'],fit_method));
    end
    if~(isnumeric(g_constant))
        error(sprintf('Input of argument ''g_constant''= ''%s'' is not numerical argument object',g_constant));
    elseif(g_constant<0)
        error(sprintf('Input of argument ''g_constant''= %d should not be negative',g_constant));
    elseif(g_constant>20)
        error(sprintf('Input of argument ''g_constant''= %d is larger than 20, please consider if it is reasonable.',g_constant));    
    end
    
    % Reading JSON Config file
    fid = fopen(GC_controller_config_json_str);
    raw = fread(fid, inf);
    str = char(raw');
    fclose(fid);
    config = jsondecode(str);

    % General Setting
    arm_name_string = GC_ARM;
    safe_upper_torque_limit = config.GC_controller.safe_upper_torque_limit;
    safe_lower_torque_limit = config.GC_controller.safe_lower_torque_limit;
    amplitude_vec = config.GC_controller.amplitude_vec;
    Zero_Output_Joint_No = [];
    beta_vel_amplitude = config.GC_controller.beta_vel_amplitude;
    g = g_constant;        

    % Load the dynamic paramters for GC Controller
    if strcmp(arm_name_string,'MTML') | strcmp(arm_name_string, 'MTML&MTMR')    
        pub_tor_MTML = rospublisher('/dvrk/MTML/set_effort_joint');
        % Load MTML GC Param
        if ischar(dynamic_paramters_root_path_or_matrix)
            root_path = dynamic_paramters_root_path_or_matrix;
            MTML_pos_file = strcat(root_path,'/MTML/Train_Joint1/data_pos/',fit_method,'/param.mat');
            MTML_neg_file = strcat(root_path,'/MTML/Train_Joint1/data_neg/',fit_method,'/param.mat');
            if exist(MTML_pos_file)~=2
                error(sprintf('Cannot find the MTML positive dynamic param file: %s', MTML_pos_file))
            end
            load(MTML_pos_file);
            param_keys = cell2mat(output_param_map.keys);
            param_values = cell2mat(output_param_map.values);
            dynamic_param_pos_MTML = zeros(size(param_keys,2),1);
            dynamic_param_pos_MTML(param_keys) = param_values;
            if exist(MTML_neg_file)~=2
                error(sprintf('Cannot find the MTML negative dynamic param file: %s', MTML_neg_file))
            end
            load(MTML_neg_file);
            param_keys = cell2mat(output_param_map.keys);
            param_values = cell2mat(output_param_map.values);
            dynamic_param_neg_MTML = zeros(size(param_keys,2),1);
            dynamic_param_neg_MTML(param_keys) = param_values;
        else
            if(size(dynamic_paramters_root_path_or_matrix,2)==2 & strcmp(arm_name_string,'MTML') |...
                size(dynamic_paramters_root_path_or_matrix,2)==4 & strcmp(arm_name_string,'MTML&MTMR'))
                dynamic_param_pos_MTML = cell2mat(dynamic_paramters_root_path_or_matrix(:,1));
                dynamic_param_neg_MTML = cell2mat(dynamic_paramters_root_path_or_matrix(:,2));
            else
                error(sprintf(['Input of argument ''dynamic_paramters_root_path_or_matrix'' has wrong row number %d,',...
                              ' should be 2 for MTML / MTMR or 4 for MTML&MTMR']...
                              ,size(dynamic_paramters_root_path_or_matrix,2)));
            end
        end
        disp('GC dynamic parameters of MTML: [pos_param], [neg_param]');
        for i=1:dynamic_param_num
            disp(sprintf('Param_%d: [%0.3f], [%0.3f]', i, dynamic_param_pos_MTML(i), dynamic_param_neg_MTML(i)));
        end
    end
 
    if strcmp(arm_name_string, 'MTMR') | strcmp(arm_name_string, 'MTML&MTMR')       
        sub_pos_MTMR = rossubscriber('/dvrk/MTMR/state_joint_current');
        pub_tor_MTMR = rospublisher('/dvrk/MTMR/set_effort_joint');
        % Load MTMR GC Param
        if ischar(dynamic_paramters_root_path_or_matrix)
            root_path = dynamic_paramters_root_path_or_matrix;
            MTMR_pos_file = strcat(root_path,'/MTMR/Train_Joint1/data_pos/',fit_method,'/param.mat');
            MTMR_neg_file = strcat(root_path,'/MTMR/Train_Joint1/data_neg/',fit_method,'/param.mat');
            if exist(MTMR_pos_file)~=2
                error(sprintf('Cannot find the MTMR positive dynamic param file: %s', MTMR_pos_file))
            end
            load(MTMR_pos_file);
            param_keys = cell2mat(output_param_map.keys);
            param_values = cell2mat(output_param_map.values);
            dynamic_param_pos_MTMR = zeros(size(param_keys,2),1);
            dynamic_param_pos_MTMR(param_keys) = param_values;
            if exist(MTMR_neg_file)~=2
                error(sprintf('Cannot find the MTMR negtive dynamic param file: %s', MTMR_neg_file))
            end
            load(MTMR_neg_file);
            param_keys = cell2mat(output_param_map.keys);
            param_values = cell2mat(output_param_map.values);
            dynamic_param_neg_MTMR = zeros(size(param_keys,2),1);
            dynamic_param_neg_MTMR(param_keys) = param_values;
        else
            if(size(dynamic_paramters_root_path_or_matrix,2)==2 & strcmp(arm_name_string,'MTMR') |...
                size(dynamic_paramters_root_path_or_matrix,2)==4 & strcmp(arm_name_string,'MTML&MTMR'))
                dynamic_param_pos_MTMR = cell2mat(dynamic_paramters_root_path_or_matrix(:,end-1));
                dynamic_param_neg_MTMR = cell2mat(dynamic_paramters_root_path_or_matrix(:,end));
            else
               error(sprintf('Input of argument ''dynamic_paramters_root_path_or_matrix'' has wrong row number %d,',...
                             ' should be 2 for MTML or MTMR or 4 for MTML&MTMR',...
                             size(dynamic_paramters_root_path_or_matrix,2)));             
            end           
        end
    end

    % % Spawn GC Controllers and test
    if strcmp(arm_name_string,'MTML') | strcmp(arm_name_string, 'MTML&MTMR') 
        mtml_arm = mtm('MTML');
        MTML_GC_Controller= Controller(pub_tor_MTML,...
                                  dynamic_param_pos_MTML,...
                                  dynamic_param_neg_MTML,...
                                  safe_upper_torque_limit,...
                                  safe_lower_torque_limit,...
                                  beta_vel_amplitude,...
                                  g,...
                                  Zero_Output_Joint_No,...
                                  mtml_arm,...
                              	  amplitude_vec,...
                                  arm_name_string);
        % Test Controller
        disp(sprintf('Testing GC controller of %s performance...',arm_name_string));
        sub_pos_MTML = rossubscriber('/dvrk/MTML/state_joint_current');
        MTML_GC_Controller.assign_subcriber(sub_pos_MTML);
        pos_name_cell = fieldnames(config.GC_controller.GC_test);
        test_pos_mat = [];
        abs_err_mat_MTML = [];
        rel_err_mat_MTML = [];
        for k=1:size(pos_name_cell)
            test_pos_mat = [test_pos_mat,getfield(config.GC_controller.GC_test,pos_name_cell{k})];
            [abs_err_MTML, rel_err_MTML] = MTML_GC_Controller.controller_test(deg2rad(test_pos_mat(:,end)));
            abs_err_mat_MTML= [abs_err_mat_MTML,abs_err_MTML];
            rel_err_mat_MTML = [rel_err_mat_MTML,rel_err_MTML];
        end
        GC_Controllers.abs_err_MTML = abs_err_MTML;
        GC_Controllers.rel_err_MTML = rel_err_MTML;
        disp(sprintf('===================='));
        disp(sprintf('Test Result for MTML'));
        for j=1:size(pos_name_cell)
        disp(sprintf('For Pose_%d Joint_No: [''absolute error''], [''error rate%%'']',j));
            for k = 1:7
                disp(sprintf('Joint%d:[%.4f], [%d%%]',k, abs_err_mat_MTML(k,j), int32(rel_err_mat_MTML(k,j)*100)));
            end
        end
        for j=1:size(pos_name_cell)
            for k = 1:7
                if(int32(rel_err_mat_MTML(k,j)*100)>=config.GC_controller.GC_test_error_rate_threshold)
                    error(sprintf('[Test Pos %d]: --MTML Joint%d-- absolute torque error:[%.4f], error rate:[%d%%], has exceed the error rate threshold %d%%',j,k,...
                                                    abs_err_mat_MTML(k,j), int32(rel_err_mat_MTML(k,j)*100),...
                                                    config.GC_controller.GC_test_error_rate_threshold));
                end
            end
        end
        MTML_GC_Controller.mtm_arm.move_joint(deg2rad(config.GC_controller.GC_init_pos));
        pause(2.5); %Wait until stable
    end
    if strcmp(arm_name_string,'MTMR') | strcmp(arm_name_string, 'MTML&MTMR')   
        mtmr_arm = mtm('MTMR');
        MTMR_GC_Controller= Controller(pub_tor_MTMR,...
                                  dynamic_param_pos_MTMR,...
                                  dynamic_param_neg_MTMR,...
                                  safe_upper_torque_limit,...
                                  safe_lower_torque_limit,...
                                  beta_vel_amplitude,...
                                  g,...
                                  Zero_Output_Joint_No,...
                                  mtmr_arm,...
                                  amplitude_vec,...
                                  arm_name_string);
        % Test Controller
        disp(sprintf('Testing GC controller of %s performance...',arm_name_string));
        sub_pos_MTMR = rossubscriber('/dvrk/MTMR/state_joint_current');
        MTMR_GC_Controller.assign_subcriber(sub_pos_MTMR);
        pos_name_cell = fieldnames(config.GC_controller.GC_test);
        test_pos_mat = [];
        abs_err_mat_MTMR = [];
        rel_err_mat_MTMR = [];
        for k=1:size(pos_name_cell)
            test_pos_mat = [test_pos_mat,getfield(config.GC_controller.GC_test,pos_name_cell{k})];
            [abs_err_MTMR, rel_err_MTMR] = MTMR_GC_Controller.controller_test(deg2rad(test_pos_mat(:,end)));
            abs_err_mat_MTMR= [abs_err_mat_MTMR,abs_err_MTMR];
            rel_err_mat_MTMR = [rel_err_mat_MTMR,rel_err_MTMR];
        end
        GC_Controllers.abs_err_MTMR = abs_err_MTMR;
        GC_Controllers.rel_err_MTMR = rel_err_MTMR;
        disp(sprintf('===================='));
        disp(sprintf('Test Result for MTMR'));
        for j=1:size(pos_name_cell)
        disp(sprintf('For Pose_%d Joint_No: [''absolute error''], [''error rate%%'']',j));
            for k = 1:7
                disp(sprintf('Joint%d:[%.4f], [%d%%]',k, abs_err_mat_MTMR(k,j), int32(rel_err_mat_MTMR(k,j)*100)));
            end
        end
        for j=1:size(pos_name_cell)
            for k = 1:7
                if(int32(rel_err_mat_MTMR(k,j)*100)>=config.GC_controller.GC_test_error_rate_threshold)
                    error(sprintf('[Test Pos %d]: --MTMR Joint%d-- absolute torque error:[%.4f], error rate:[%d%%], has exceed the error rate threshold %d%%',j,k,...
                                                    abs_err_mat_MTMR(k,j), int32(rel_err_mat_MTMR(k,j)*100),...
                                                    config.GC_controller.GC_test_error_rate_threshold));
                end
            end
        end
        MTMR_GC_Controller.mtm_arm.move_joint(deg2rad(config.GC_controller.GC_init_pos));
        pause(2.5); %Wait until stable
    end

    
    % Apply GC Controllers
     if strcmp(arm_name_string,'MTML') | strcmp(arm_name_string, 'MTML&MTMR') 
        callback_MTML = @(src,msg)(MTML_GC_Controller.callback_gc_publisher(msg.Position,...
                                                                           msg.Velocity));
        sub_pos_MTML = rossubscriber('/dvrk/MTML/state_joint_current',callback_MTML,'BufferSize',10); 
        MTML_GC_Controller.assign_subcriber(sub_pos_MTML);
        
        % Assign output struct
        GC_Controllers.MTML = MTML_GC_Controller;
     end
     if strcmp(arm_name_string,'MTMR') | strcmp(arm_name_string, 'MTML&MTMR')   
             callback_MTMR = @(src,msg)(MTMR_GC_Controller.callback_gc_publisher(msg.Position,...
                                                                           msg.Velocity));
        sub_pos_MTMR = rossubscriber('/dvrk/MTMR/state_joint_current',callback_MTMR,'BufferSize',100); 
        MTMR_GC_Controller.assign_subcriber(sub_pos_MTMR);
        
        % Assign output struct
        GC_Controllers.MTMR = MTMR_GC_Controller;
     end
end
