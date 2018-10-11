function [is_MTM_lse_finish, output_dynamic_matrix, output_lse_config] = MLSE(GC_ARM, LSE_json_str, input_data_root_path)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05

    % General Setting
    output_dynamic_matrix = [];
    is_MTM_lse_finish.MTML = false;
    is_MTM_lse_finish.MTMR = false;
    arm_name_string = GC_ARM;
    disp(sprintf('input_data_root_path for MLSE : ''%s'' ',input_data_root_path));
    disp(' ');
    
    % Argument Checking
    
    
    % Read JSON Config File
    fid = fopen(LSE_json_str);
    raw = fread(fid, inf);
    str = char(raw');
    fclose(fid);
    config = jsondecode(str);
    
    % output_lse_config assignment
    output_lse_config.symbolic_file = config.lse.symbolic_file;
    output_lse_config.output_data_root_path = input_data_root_path; 
    output_lse_config.symbolic_file = config.lse.symbolic_file;
    output_lse_config.std_filter = config.lse.std_filter;
    output_lse_config.g_constant = config.lse.g_constant;
    output_lse_config.fit_method = config.lse.joint1.fit_method; % Joint 1 is the last step for LSE and its paramters is also output param.
    
    % Multi-Step LSE Process
    if strcmp(arm_name_string, 'MTML') | strcmp(arm_name_string, 'MTML&MTMR')
        arm_name = 'MTML';
        [config_lse_joint1, config_lse_joint2_3,config_lse_joint4,config_lse_joint5,config_lse_joint6]=...
                            setting_LSE(arm_name,LSE_json_str,input_data_root_path);

        [is_lse_success] = LSE_MTM_one_joint(config_lse_joint6);

        if(is_lse_success)
            [is_lse_success] = LSE_MTM_one_joint(config_lse_joint5, config_lse_joint6);
        end
        if(is_lse_success)
            [is_lse_success] = LSE_MTM_one_joint(config_lse_joint4, config_lse_joint5);
        end
        if(is_lse_success)
            [is_lse_success] = LSE_MTM_one_joint(config_lse_joint2_3 ,config_lse_joint4);
        end
        
        if(is_lse_success)
            [is_lse_success,  MTML_output_dynamic_matrix] = LSE_MTM_one_joint(config_lse_joint1 ,config_lse_joint2_3);
            is_MTM_lse_finish.MTML = is_lse_success;
            % size(output_dynamic_matrix)=(41,2) for MTML or
            % size(output_dynamic_matrix)=(41,4) for MTML&MTMR
            output_dynamic_matrix = [output_dynamic_matrix,MTML_output_dynamic_matrix];
        end
    end
    
    if strcmp(arm_name_string, 'MTMR') | strcmp(arm_name_string, 'MTML&MTMR')
        arm_name = 'MTMR';
        [config_lse_joint1, config_lse_joint2_3,config_lse_joint4,config_lse_joint5,config_lse_joint6]=...
                            setting_LSE(arm_name,LSE_json_str,input_data_root_path);

        [is_lse_success] = LSE_MTM_one_joint(config_lse_joint6);

        if(is_lse_success)
            [is_lse_success] = LSE_MTM_one_joint(config_lse_joint5, config_lse_joint6);
        end
        if(is_lse_success)
            [is_lse_success] = LSE_MTM_one_joint(config_lse_joint4, config_lse_joint5);
        end
        if(is_lse_success)
            [is_lse_success] = LSE_MTM_one_joint(config_lse_joint2_3 ,config_lse_joint4);
        end
        
        if(is_lse_success)
            [is_lse_success,  MTMR_output_dynamic_matrix] = LSE_MTM_one_joint(config_lse_joint1 ,config_lse_joint2_3);
            is_MTM_lse_finish.MTMR = is_lse_success;
            % size(output_dynamic_matrix)=(41,2) for MTMR or
            % size(output_dynamic_matrix)=(41,4) for MTML&MTMR
            output_dynamic_matrix = [output_dynamic_matrix,MTMR_output_dynamic_matrix];
        end
    end
 
end