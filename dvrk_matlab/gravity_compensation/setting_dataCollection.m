function [config_joint1, config_joint2_3,config_joint4,config_joint5,config_joint6]= setting_dataCollection(arm_name,json_file)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05

    % Reading the json file                       
    fid = fopen(json_file);
    raw = fread(fid, inf);
    str = char(raw');
    fclose(fid);
    config = jsondecode(str);
    
    % Genneral Setting
    is_path_planning = config.data_collection.is_path_planning; 
    is_collecting_data = config.data_collection.is_collecting_data;
    root_data_path = config.data_collection.output_data_root_path; 
    sample_num = config.data_collection.sample_num;
    is_pos_dir = config.data_collection.is_pos_dir;
    is_neg_dir = config.data_collection.is_neg_dir;
    
    %% Data collection setting of Joint6 
    % User Setting
    train_angle_list = deg2rad(config.data_collection.joint6.train_angle_min:config.data_collection.joint6.train_angle_delta:config.data_collection.joint6.train_angle_max); %train_angle_list = deg2rad(-40:2:40);
    theta_angle_list = deg2rad(config.data_collection.joint6.theta_angle_min:config.data_collection.joint6.theta_angle_delta:config.data_collection.joint6.theta_angle_max); %theta_angle_list = deg2rad(-90:30:90);
    init_joint_range= deg2rad(config.data_collection.joint6.init_joint_range);
    Theta_Joint_No = config.data_collection.joint6.Theta_Joint_No;
    Train_Joint_No = config.data_collection.joint6.Train_Joint_No;
    config_joint6 =  config_collecting(Theta_Joint_No, theta_angle_list,...
                        Train_Joint_No, train_angle_list, arm_name,...
                        root_data_path,sample_num,is_path_planning, is_neg_dir,is_collecting_data, init_joint_range,is_pos_dir);

    %% Data collection setting of Joint5
    % User Setting
    train_angle_list = deg2rad(config.data_collection.joint5.train_angle_min:config.data_collection.joint5.train_angle_delta:config.data_collection.joint5.train_angle_max); %train_angle_list = deg2rad(-40:2:40);
    theta_angle_list = deg2rad(config.data_collection.joint5.theta_angle_min:config.data_collection.joint5.theta_angle_delta:config.data_collection.joint5.theta_angle_max); %theta_angle_list = deg2rad(-90:30:90);
    init_joint_range=deg2rad(config.data_collection.joint5.init_joint_range);
    Theta_Joint_No = config.data_collection.joint5.Theta_Joint_No;
    Train_Joint_No = config.data_collection.joint5.Train_Joint_No;
    config_joint5 =  config_collecting(Theta_Joint_No, theta_angle_list,...
                        Train_Joint_No, train_angle_list, arm_name,...
                        root_data_path,sample_num,is_path_planning, is_neg_dir,is_collecting_data, init_joint_range,is_pos_dir);
  
    %% Data collection setting of Joint4
    % User Setting    
    if arm_name == 'MTML'
        train_angle_list = deg2rad(config.data_collection.joint4.train_angle_min.MTML:config.data_collection.joint4.train_angle_delta:config.data_collection.joint4.train_angle_max.MTML); 
    elseif arm_name == 'MTMR'
        train_angle_list = deg2rad(config.data_collection.joint4.train_angle_min.MTMR:config.data_collection.joint4.train_angle_delta:config.data_collection.joint4.train_angle_max.MTMR); 
    end
    theta_angle_list = deg2rad(config.data_collection.joint4.theta_angle_min:config.data_collection.joint4.theta_angle_delta:config.data_collection.joint4.theta_angle_max); %theta_angle_list = deg2rad(-90:30:90);
    init_joint_range=deg2rad(config.data_collection.joint4.init_joint_range);
    Theta_Joint_No = config.data_collection.joint4.Theta_Joint_No;
    Train_Joint_No = config.data_collection.joint4.Train_Joint_No;
    config_joint4 =  config_collecting(Theta_Joint_No, theta_angle_list,...
                        Train_Joint_No, train_angle_list, arm_name,...
                        root_data_path,sample_num,is_path_planning, is_neg_dir,is_collecting_data, init_joint_range,is_pos_dir);

    %% Data collection setting of Joint3
    % User Setting
    train_angle_list = deg2rad(config.data_collection.joint2_3.train_angle_min:config.data_collection.joint2_3.train_angle_delta:config.data_collection.joint2_3.train_angle_max); 
    theta_angle_list = deg2rad(config.data_collection.joint2_3.theta_angle_min:config.data_collection.joint2_3.theta_angle_delta:config.data_collection.joint2_3.theta_angle_max); 
    Theta_Joint_No = config.data_collection.joint2_3.Theta_Joint_No;
    Train_Joint_No = config.data_collection.joint2_3.Train_Joint_No;
    if arm_name == 'MTML'
        init_joint_range=deg2rad(config.data_collection.joint2_3.init_joint_range.MTML); % init_joint_range = {deg2rad(28),0,0,0,0,0,0};
    elseif arm_name == 'MTMR'
        init_joint_range=deg2rad(config.data_collection.joint2_3.init_joint_range.MTMR); % init_joint_range = {deg2rad(-28),0,0,0,0,0,0};
    end
    % Defalut Setting
    config_joint2_3 =  config_collecting(Theta_Joint_No, theta_angle_list,...
                        Train_Joint_No, train_angle_list, arm_name,...
                        root_data_path,sample_num,is_path_planning, is_neg_dir,is_collecting_data, init_joint_range,is_pos_dir);
    % User Setting
    config_joint2_3 = config_joint2_3.set_couple_upper_limit(deg2rad(config.data_collection.joint2_3.couple_upper_limit)); 
    config_joint2_3 = config_joint2_3.set_couple_lower_limit(deg2rad(config.data_collection.joint2_3.couple_lower_limit));

    %% Data collection setting of Joint1
    % User Setting
    if arm_name == 'MTML'
        train_angle_list = deg2rad(config.data_collection.joint1.train_angle_min.MTML:config.data_collection.joint1.train_angle_delta:config.data_collection.joint1.train_angle_max.MTML);
    elseif arm_name == 'MTMR'
        train_angle_list = deg2rad(config.data_collection.joint1.train_angle_min.MTMR:config.data_collection.joint1.train_angle_delta:config.data_collection.joint1.train_angle_max.MTMR);
    end
    Theta_Joint_No = config.data_collection.joint1.Theta_Joint_No;
    theta_angle_list = 0;
    init_joint_range = deg2rad([0;0;0;0;0;0;0]);
    Train_Joint_No = config.data_collection.joint1.Train_Joint_No;
    config_joint1 =  config_collecting(Theta_Joint_No, theta_angle_list,...
                        Train_Joint_No, train_angle_list, arm_name,...
                        root_data_path,sample_num,is_path_planning, is_neg_dir,is_collecting_data, init_joint_range,is_pos_dir);

end