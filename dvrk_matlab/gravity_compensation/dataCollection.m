function [is_dataCollection_finish, output_data_struct] = dataCollection(GC_ARM, dataCollection_config_json)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05

    clc;
    % Read JSON Config File
    disp(sprintf('Loading JSON File %s', dataCollection_config_json));
    fid = fopen(dataCollection_config_json);
    raw = fread(fid, inf);
    str = char(raw');
    fclose(fid);
    config = jsondecode(str);
    
    % General Setting
    arm_name_string = GC_ARM;
    is_dataCollection_finish = false;
    output_data_struct.output_data_root_path = config.data_collection.output_data_root_path;
    is_path_planning = config.data_collection.is_path_planning;
    is_collecting_data = config.data_collection.is_collecting_data;
    
    % behavior_buff to store the path planning and collecting data order,
    %   |behavor_vector_1 = [is_path_planning; is_collecting_data]
    %   |behavior_buff = [behavor_vector_1,behavor_vector_2,...]
    if(is_path_planning&is_collecting_data)
        behavior_vector1 = [1;0];
        behavior_vector2 = [0;1];
        behavior_buff = [behavior_vector1,behavior_vector2];
    else
        behavior_buff = [is_path_planning; is_collecting_data];
    end
    
    % Check if there is old data sets path, user need to choose if you want
    % to cover
    if exist(output_data_struct.output_data_root_path)==7
        input_str = '';
        clc;
        while(~strcmp(input_str,'y') & ~strcmp(input_str,'n'))
            clc
            input_str = input(sprintf('Found path:[%s] already exist, do you want to cover y/n:  ',...
                                        output_data_struct.output_data_root_path),'s');
        end
        if input_str == 'y'
            rmdir(config.data_collection.output_data_root_path,'s');
        elseif input_str == 'n'
            input_str = '';
            while(~strcmp(input_str,'m') & ~strcmp(input_str,'s') & ~strcmp(input_str,'q'))
                clc;
                disp(sprintf('Which action do you want to take?'));
                disp(sprintf('merge and save new data with old data, and use them to implement LSE together. [m]'));
                disp(sprintf('Skip data collection process and implement LSE. [s]'));
                input_str = input(sprintf('quit GC process. [q]: '),'s');
            end
            if input_str == 'q'
                is_dataCollection_finish = false;
                clc
                disp(sprintf('quiting data collection process..'));
                return 
            elseif input_str == 's'
                clc
                disp(sprintf('Skiping data collection process, Identify paramters with old data'));
                is_dataCollection_finish = true;
                return 
            end
        end
    end
    
    % Multi-steps Data collection
    for l = 1:size(behavior_buff,2)
        if strcmp(arm_name_string,'MTML') | strcmp(arm_name_string, 'MTML&MTMR')
            arm_name_L = 'MTML';
            mtml_arm = mtm(arm_name_L);
            mtml_arm.move_joint([0,0,0,0,0,0,0]);
            [config_joint1, config_joint2_3,config_joint4,config_joint5,config_joint6]= setting_dataCollection(arm_name_L,dataCollection_config_json);
            output_data_struct.MTML.joint6 = collect_mtm_torques(config_joint6, mtml_arm, behavior_buff(1,l), behavior_buff(2,l));
            output_data_struct.MTML.joint5 = collect_mtm_torques(config_joint5, mtml_arm, behavior_buff(1,l), behavior_buff(2,l));
            output_data_struct.MTML.joint4 = collect_mtm_torques(config_joint4, mtml_arm, behavior_buff(1,l), behavior_buff(2,l));
            output_data_struct.MTML.joint3 = collect_mtm_torques(config_joint2_3, mtml_arm, behavior_buff(1,l), behavior_buff(2,l));
            output_data_struct.MTML.joint1 = collect_mtm_torques(config_joint1, mtml_arm, behavior_buff(1,l), behavior_buff(2,l));
            mtml_arm.move_joint([0,0,0,0,0,0,0]);
            
        end
        if strcmp(arm_name_string, 'MTMR') | strcmp(arm_name_string, 'MTML&MTMR')    
            arm_name_R = 'MTMR';
            mtmr_arm = mtm(arm_name_R);
            mtmr_arm.move_joint([0,0,0,0,0,0,0]);
            [config_joint1, config_joint2_3,config_joint4,config_joint5,config_joint6]= setting_dataCollection(arm_name_R,dataCollection_config_json);
            output_data_struct.MTMR.joint6 = collect_mtm_torques(config_joint6, mtmr_arm, behavior_buff(1,l), behavior_buff(2,l));
            output_data_struct.MTMR.joint5 = collect_mtm_torques(config_joint5, mtmr_arm, behavior_buff(1,l), behavior_buff(2,l));
            output_data_struct.MTMR.joint4 = collect_mtm_torques(config_joint4, mtmr_arm, behavior_buff(1,l), behavior_buff(2,l));
            output_data_struct.MTMR.joint3 = collect_mtm_torques(config_joint2_3, mtmr_arm, behavior_buff(1,l), behavior_buff(2,l));
            output_data_struct.MTMR.joint1 = collect_mtm_torques(config_joint1, mtmr_arm, behavior_buff(1,l), behavior_buff(2,l));
            mtmr_arm.move_joint([0,0,0,0,0,0,0]);
        end
    end
    
    is_dataCollection_finish = true;
end