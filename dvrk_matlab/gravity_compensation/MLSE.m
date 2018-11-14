function [is_MTM_lse_finish, output_dynamic_matrix, output_lse_config] = MLSE(GC_ARM, LSE_json_str, input_data_root_path)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05
%  Output Arguments: 
%       [is_MTM_lse_finish]: true/false
%       [output_dynamic_matrix]: MTML: MTML_dynamic_vector | MTMR: MTMR_dynamic_vector | MTML&MTMR: [MTML_dynamic_vector,MTMR_dynamic_vector]
%       [output_lse_config]: config from 'LSE_json_str' JSON file

    % general setting
    output_dynamic_matrix = [];
    is_MTM_lse_finish.MTML = false;
    is_MTM_lse_finish.MTMR = false;
    arm_name_string = GC_ARM;
    
    % display data input root path
    disp(' ');
    disp(sprintf('input_data_root_path for MLSE : ''%s'' ',input_data_root_path));
    disp(' ');
   
    % read JSON config file
    fid = fopen(LSE_json_str);
    raw = fread(fid, inf);
    str = char(raw');
    fclose(fid);
    config = jsondecode(str);
    
    % output output_lse_config 
    output_lse_config.output_data_root_path = input_data_root_path; 
    output_lse_config.std_filter = config.lse.std_filter;
    output_lse_config.g_constant = config.lse.g_constant;
    % GC_controller will read the fit_method of Joint1 at the stage where Multi-steps LSE terminates.
    output_lse_config.fit_method = config.lse.joint1.fit_method; 
    
    % Multi-Step LSE process
    if strcmp(arm_name_string, 'MTML') | strcmp(arm_name_string, 'MTML&MTMR')
        arm_name = 'MTML';
        
        % create config_LSE objs
        [config_lse_joint1,config_lse_joint2, config_lse_joint3,config_lse_joint4,config_lse_joint5,config_lse_joint6]=...
                            setting_LSE(arm_name,LSE_json_str,input_data_root_path);
         
       % It will go through from Joint6 to Joint1.
        LSE_MTM_one_joint(config_lse_joint6);
        LSE_MTM_one_joint(config_lse_joint5, config_lse_joint6);
        LSE_MTM_one_joint(config_lse_joint4, config_lse_joint5);
        LSE_MTM_one_joint(config_lse_joint3 ,config_lse_joint4);
        LSE_MTM_one_joint(config_lse_joint2 ,config_lse_joint3);
        MTML_output_dynamic_matrix = LSE_MTM_one_joint(config_lse_joint1 ,config_lse_joint2);
        
        % output parameters
        output_dynamic_matrix = [output_dynamic_matrix,MTML_output_dynamic_matrix]; 
        is_MTM_lse_finish.MTML = true;
    end
    
    if strcmp(arm_name_string, 'MTMR') | strcmp(arm_name_string, 'MTML&MTMR')
        arm_name = 'MTMR';
        
        % create config_LSE objs
        [config_lse_joint1,config_lse_joint2, config_lse_joint3,config_lse_joint4,config_lse_joint5,config_lse_joint6]=...
                            setting_LSE(arm_name,LSE_json_str,input_data_root_path);
         
       % It will go through from Joint6 to Joint1.
        LSE_MTM_one_joint(config_lse_joint6);
        LSE_MTM_one_joint(config_lse_joint5, config_lse_joint6);
        LSE_MTM_one_joint(config_lse_joint4, config_lse_joint5);
        LSE_MTM_one_joint(config_lse_joint3 ,config_lse_joint4);
        LSE_MTM_one_joint(config_lse_joint2 ,config_lse_joint3);
        MTMR_output_dynamic_matrix = LSE_MTM_one_joint(config_lse_joint1 ,config_lse_joint2);
        
        % output parameters
        output_dynamic_matrix = [output_dynamic_matrix,MTMR_output_dynamic_matrix]; 
        is_MTM_lse_finish.MTMR = true;
    end
 
end