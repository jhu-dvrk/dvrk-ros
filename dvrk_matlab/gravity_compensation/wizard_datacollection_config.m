function output_json_str = wizard_datacollection_config(GC_ARM, input_json_str)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05
    
    % checking if arguments correct
    argument_checking(GC_ARM, input_json_str);
    
    % Read JSON File
    fid = fopen(input_json_str);
    raw = fread(fid, inf);
    str = char(raw');
    config = jsondecode(str);
    fclose(fid);


    % General Setting
    if strcmp(GC_ARM,'MTML') | strcmp(GC_ARM, 'MTML&MTMR')
        arm_name_L = 'MTML';
        mtml_arm = mtm(arm_name_L);
        mtml_arm.move_joint([0,0,0,0,0,0,0]);
    end
    if strcmp(GC_ARM, 'MTMR') | strcmp(GC_ARM, 'MTML&MTMR') 
        arm_name_R = 'MTMR';
        mtmr_arm = mtm(arm_name_R);
        mtmr_arm.move_joint([0,0,0,0,0,0,0]);
    end
    
 
    % Specifying the front panel paramter
    if strcmp(GC_ARM,'MTML') | strcmp(GC_ARM, 'MTML&MTMR')
        arm_string = 'MTML';
        mtm_arm = mtml_arm;
        joint_init_pos = config.data_collection.joint3.init_joint_range.MTML;
    else
        arm_string = 'MTMR';
        mtm_arm = mtmr_arm;
        joint_init_pos = config.data_collection.joint3.init_joint_range.MTMR;
    end
    
    Joint_No = 2;
    param_name = 'theta_angle_max';
    recommend_value = config.data_collection.joint3.theta_angle_max;
    joint_init_pos(2) = recommend_value-10; %10 degree smaller for saftey reason
    joint_init_pos(3) = config.data_collection.joint3.train_angle_min;
    goal_msg = 'Moving up MTM ARM as close to front panel as possible..';
    config.data_collection.joint3.theta_angle_max = wizard_move_one_joint(mtm_arm,...
                                                                               joint_init_pos,...
                                                                               Joint_No,...
                                                                               param_name,...
                                                                               arm_string,...
                                                                               recommend_value,...
                                                                               goal_msg);
    
    % Specifying the upper panel paramter
    if strcmp(GC_ARM,'MTML') | strcmp(GC_ARM, 'MTML&MTMR')
        arm_string = 'MTML';
        mtm_arm = mtml_arm;
        joint_init_pos = config.data_collection.joint3.init_joint_range.MTML;
    else
        arm_string = 'MTMR';
        mtm_arm = mtmr_arm;
        joint_init_pos = config.data_collection.joint3.init_joint_range.MTMR;
    end
    
    Joint_No = 3;
    joint_init_pos(2) = config.data_collection.joint3.theta_angle_max;
    joint_init_pos(3) = config.data_collection.joint3.train_angle_min;
    param_name = 'couple_upper_limit';
    recommend_value = config.data_collection.joint3.couple_upper_limit;
    goal_msg = 'Moving up MTM ARM as close to upper panel as possible..';
    config.data_collection.joint3.couple_upper_limit = wizard_move_one_joint(mtm_arm,...
                                                                               joint_init_pos,...
                                                                               Joint_No,...
                                                                               param_name,...
                                                                               arm_string,...
                                                                               recommend_value,...
                                                                               goal_msg,...
                                                                               true);

    % Specifying the MTML left side panel paramter
     if strcmp(GC_ARM,'MTML') | strcmp(GC_ARM, 'MTML&MTMR')
        arm_string = 'MTML';
        mtm_arm = mtml_arm;
        joint_init_pos = zeros(1,7);

        Joint_No = 1;
        param_name = 'train_angle_min';
        recommend_value = config.data_collection.joint1.train_angle_min.MTML;
        goal_msg = 'Moving up MTML ARM as close to left side panel as possible..';
        config.data_collection.joint1.train_angle_min.MTML = wizard_move_one_joint(mtm_arm,...
                                                                                   joint_init_pos,...
                                                                                   Joint_No,...
                                                                                   param_name,...
                                                                                   arm_string,...
                                                                                   recommend_value,...
                                                                                   goal_msg); 
        mtm_arm.move_joint(zeros(1,7));
     end

     % Specifying the MTMR right side panel paramter
     if strcmp(GC_ARM,'MTMR') | strcmp(GC_ARM, 'MTML&MTMR')
        arm_string = 'MTMR';
        mtm_arm = mtmr_arm;
        joint_init_pos = zeros(1,7);

        Joint_No = 1;
        param_name = 'train_angle_max';
        recommend_value = config.data_collection.joint1.train_angle_max.MTMR;
        goal_msg = 'Moving up MTMR ARM as close to right side panel as possible..';
        config.data_collection.joint1.train_angle_max.MTMR = wizard_move_one_joint(mtm_arm,...
                                                                                   joint_init_pos,...
                                                                                   Joint_No,...
                                                                                   param_name,...
                                                                                   arm_string,...
                                                                                   recommend_value,...
                                                                                   goal_msg); 
        mtm_arm.move_joint(zeros(1,7));
     end
     
    % Specifying the MTML right side panel paramter
     if strcmp(GC_ARM,'MTML') | strcmp(GC_ARM, 'MTML&MTMR')
        arm_string = 'MTML';
        mtm_arm = mtml_arm;
        joint_init_pos = zeros(1,7);
        joint_init_pos(1) = 30;
        Joint_No = 1;
        param_name = 'train_angle_max';
        recommend_value = config.data_collection.joint1.train_angle_max.MTML;
        goal_msg = 'Moving up MTML ARM as close to right side panel as possible, without hitting MTMR..';
        config.data_collection.joint1.train_angle_max.MTML = wizard_move_one_joint(mtm_arm,...
                                                                                   joint_init_pos,...
                                                                                   Joint_No,...
                                                                                   param_name,...
                                                                                   arm_string,...
                                                                                   recommend_value,...
                                                                                   goal_msg);  
        mtm_arm.move_joint(zeros(1,7));
     end
     
     % Specifying the MTMR left side panel paramter
     if strcmp(GC_ARM,'MTMR') | strcmp(GC_ARM, 'MTML&MTMR')
        arm_string = 'MTMR';
        mtm_arm = mtmr_arm;
        joint_init_pos = zeros(1,7);
        joint_init_pos(1) = -30;
        Joint_No = 1;
        param_name = 'train_angle_min';
        recommend_value = config.data_collection.joint1.train_angle_min.MTMR;
        goal_msg = 'Moving up MTMR ARM as close to left side panel as possible..';
        config.data_collection.joint1.train_angle_min.MTMR = wizard_move_one_joint(mtm_arm,...
                                                                                   joint_init_pos,...
                                                                                   Joint_No,...
                                                                                   param_name,...
                                                                                   arm_string,...
                                                                                   recommend_value,...
                                                                                   goal_msg);  
        mtm_arm.move_joint(zeros(1,7));
     end    
     

     
    % Output Display
    clc;
    disp(sprintf('Your customized parameters is:'));
    disp(sprintf('joint3.theta_angle_max: %d', config.data_collection.joint3.theta_angle_max));
    disp(sprintf('joint3.couple_upper_limit: %d', config.data_collection.joint3.couple_upper_limit));
    if strcmp(GC_ARM,'MTML') | strcmp(GC_ARM, 'MTML&MTMR')
        disp(sprintf('Joint1.train_angle_min.MTML: %d', config.data_collection.joint1.train_angle_min.MTML));
        disp(sprintf('Joint1.train_angle_max.MTML: %d', config.data_collection.joint1.train_angle_max.MTML));
    end
    if strcmp(GC_ARM,'MTMR') | strcmp(GC_ARM, 'MTML&MTMR')
        disp(sprintf('Joint1.train_angle_min.MTMR: %d', config.data_collection.joint1.train_angle_min.MTMR));
        disp(sprintf('Joint1.train_angle_max.MTMR: %d', config.data_collection.joint1.train_angle_max.MTMR));
    end

    % save wizard JSON File
    file_name = [config.data_collection.output_data_root_path,'/', input_json_str(1:end-5),'_',...
        datestr(datetime('now'),'mmmm-dd-yyyy-HH-MM-SS'),'_wizard.json'];
    fid = fopen(file_name,'w');
    jsonStr = jsonencode(config);
    fwrite(fid, jsonStr);
    fclose(fid);
    disp(sprintf('Save config file to %s', file_name));
    output_json_str = file_name;
end

function customized_value = wizard_move_one_joint(mtm_arm, joint_init_pos, Joint_No, param_name, arm_string,...
                                                    recommend_value, goal_msg,is_couple_limit)
    input_str = '';
    clc;
    disp(sprintf('Moving to init pose for identifying paramter %s of Joint %d for %s',param_name,Joint_No,arm_string));
    mtm_arm.move_joint(deg2rad(joint_init_pos));
    if(~exist('is_couple_limit'))
        customized_value = joint_init_pos(Joint_No);
    else
        customized_value = joint_init_pos(Joint_No) + joint_init_pos(Joint_No-1);
    end
    joint_pos = joint_init_pos;
    while(true)
         while(~strcmp(input_str,'i') & ~strcmp(input_str,'d') & ~strcmp(input_str,'f'))
            clc
            disp(sprintf('Goal: %s', goal_msg));
            disp(sprintf('Arm: %s',arm_string));
            disp(sprintf('Joint_No: %d',Joint_No));
            disp(sprintf('Customized Param Name: %s',param_name));           
            disp(sprintf('Recommend Value: [%s] = %d degree ',param_name,recommend_value));
            disp(sprintf('Current Value: [%s] = %d degree', param_name,customized_value));
            disp('Increase the value by 1 degree: [i]');
            disp('Decrease the value by 1 degree: [d]');
            input_str = input(sprintf('Finish the process: [f]:'),'s');
         end
         if(input_str == 'i')
             customized_value = customized_value+1;      
         elseif(input_str == 'd')
             customized_value = customized_value-1;
         else
             break
         end
         if(~exist('is_couple_limit'))
            joint_pos(Joint_No) = customized_value;
         else
             joint_pos(Joint_No) = customized_value - joint_pos(Joint_No-1);
         end
         mtm_arm.move_joint(deg2rad(joint_pos));
         input_str = '';
    end
end

function argument_checking(GC_ARM, input_json_str)
    if~(strcmp(GC_ARM,'MTML') | strcmp(GC_ARM,'MTMR') | strcmp(GC_ARM,'MTML&MTMR'))
        error(sprintf(['Input of argument ''GC_ARM''= %s is error, you should input one of the string',...
                       '[''MTML'',''MTMR'',''MTML&MTMR'']'],GC_ARM));
    end
    if exist(input_json_str) ~= 2
        error(sprintf('Cannot find the json file %s',input_json_str))
    end

    if(~strcmp(input_json_str(end-4:end),'.json'))
        error(sprintf('file %s is not .json extend',input_json_str))
    end
end