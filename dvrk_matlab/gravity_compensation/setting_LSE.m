function [config_lse_joint1,config_lse_joint2, config_lse_joint3,config_lse_joint4,config_lse_joint5,config_lse_joint6]=...
                    setting_LSE(arm_name, config_json_str, data_input_root_path)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05

    %Read JSON file
    fid = fopen(config_json_str);
    raw = fread(fid, inf);
    str = char(raw');
    fclose(fid);
    config = jsondecode(str);
    
    %General Setting
    root_input_path = data_input_root_path; 
    Is_Plot = config.lse.isplot;
    issave_figure = config.lse.issave_figure;
    std_filter = config.lse.std_filter;
    g_constant = config.lse.g_constant;
    if arm_name == 'MTML'
        input_path = strcat(root_input_path,'/MTML');
    elseif arm_name == 'MTMR'
        input_path = strcat(root_input_path,'/MTMR');
    end
    
    

    %Generate config obj for lse
    Joint_No = 6;
    config_lse_joint6 =  config_lse(Joint_No,std_filter,Is_Plot,...
                  issave_figure,[input_path,'/Train_Joint6'],config.lse.joint6.fit_method,g_constant);

    Joint_No = 5;
    config_lse_joint5 =  config_lse(Joint_No,std_filter,Is_Plot,...
                  issave_figure,[input_path,'/Train_Joint5'],config.lse.joint5.fit_method,g_constant);


    Joint_No = 4;
    config_lse_joint4 =  config_lse(Joint_No,std_filter,Is_Plot,...
                  issave_figure,[input_path,'/Train_Joint4'],config.lse.joint4.fit_method,g_constant);


    Joint_No = 3;
    config_lse_joint3 =  config_lse(Joint_No,std_filter,Is_Plot,...
                  issave_figure,[input_path,'/Train_Joint3'],config.lse.joint3.fit_method,g_constant);
              
    Joint_No = 2;
    config_lse_joint2 =  config_lse(Joint_No,std_filter,Is_Plot,...
                  issave_figure,[input_path,'/Train_Joint2'],config.lse.joint2.fit_method,g_constant);


    Joint_No = 1;
    config_lse_joint1 =  config_lse(Joint_No,std_filter,Is_Plot,...
                  issave_figure,[input_path,'/Train_Joint1'],config.lse.joint1.fit_method,g_constant);

end