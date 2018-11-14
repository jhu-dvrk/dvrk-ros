function [is_dataCollection_finish, output_data_struct] = dataCollection(GC_ARM, dataCollection_config_str)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05

    % checking if arguments correct
    argument_checking(GC_ARM, dataCollection_config_str);
    
    % Read JSON Config File and check if save path already exists
    disp(sprintf('Loading JSON File %s', dataCollection_config_str));
    fid = fopen(dataCollection_config_str);
    raw = fread(fid, inf);
    str = char(raw');
    fclose(fid);
    config = jsondecode(str);
    [is_dataCollection_finish, is_quit_data_collection] = check_data_save_path(config.data_collection.output_data_root_path);
    if is_quit_data_collection & is_dataCollection_finish
        output_data_struct.output_data_root_path = config.data_collection.output_data_root_path;
        return
    elseif is_quit_data_collection & ~is_dataCollection_finish
        output_data_struct = [];    
        return
    end    

    % wizard for setting data collection config
    input_str = '';
    while(~strcmp(input_str,'y') & ~strcmp(input_str,'n'))
        disp('Do you want to start the wizard for data collection, ([y]/[n])?')
        disp(sprintf('[y]: start the wizard for data collection to set your customized setting (strongly recommended!)'));
        input_str = input(sprintf('[n]: use the default setting in File %s :',dataCollection_config_str),'s');
    end
    if input_str == 'y'
        % Pre Setting Wizard
        dataCollection_config_str = wizard_datacollection_config(GC_ARM, dataCollection_config_str);      
    end     

    
    % General Setting
    arm_name_string = GC_ARM;
    is_dataCollection_finish = false;
    output_data_struct.output_data_root_path = config.data_collection.output_data_root_path;
    is_path_planning = config.data_collection.is_path_planning;
    is_collecting_data = config.data_collection.is_collecting_data;
    
    
    
    % behavor_vector = [is_path_planning; is_collecting_data]
    % behavior_buff = [behavior_vector1,behavior_vector2,...]
    if(is_path_planning&is_collecting_data)
        %behavior_vector1 for only path_planning
        behavior_vector1 = [1;0];
        
        %behavior_vector2 for only collecting_data
        behavior_vector2 = [0;1];
        
        % The system will first run behavior_vector1 and then run behavior_vector2;
        behavior_buff = [behavior_vector1,behavior_vector2];
    else
        behavior_vector =[is_path_planning; is_collecting_data];
        behavior_buff = behavior_vector;
    end

    % create mtm_arm obj and move every arm in home position for safety reason
    if strcmp(arm_name_string,'MTML') | strcmp(arm_name_string, 'MTML&MTMR')
        arm_name_L = 'MTML';
        mtml_arm = mtm(arm_name_L);
        mtml_arm.move_joint([0,0,0,0,0,0,0]);
    end
     if strcmp(arm_name_string, 'MTMR') | strcmp(arm_name_string, 'MTML&MTMR')
        arm_name_R = 'MTMR';
        mtmr_arm = mtm(arm_name_R);
        mtmr_arm.move_joint([0,0,0,0,0,0,0]);   
     end

    
    % Multi-steps Data collection
    for l = 1:size(behavior_buff,2)
        % The system will first run path_planning and then run collecting_data if both is_path_planning and is_collecting_data are true
        if strcmp(arm_name_string,'MTML') | strcmp(arm_name_string, 'MTML&MTMR')
            [config_joint1, config_joint2, config_joint3,config_joint4,config_joint5,config_joint6]= setting_dataCollection(arm_name_L,dataCollection_config_str);
            output_data_struct.MTML.joint6 = collect_mtm_one_joint(config_joint6, mtml_arm, behavior_buff(1,l), behavior_buff(2,l));
            output_data_struct.MTML.joint5 = collect_mtm_one_joint(config_joint5, mtml_arm, behavior_buff(1,l), behavior_buff(2,l));
            output_data_struct.MTML.joint4 = collect_mtm_one_joint(config_joint4, mtml_arm, behavior_buff(1,l), behavior_buff(2,l));
            output_data_struct.MTML.joint3 = collect_mtm_one_joint(config_joint3, mtml_arm, behavior_buff(1,l), behavior_buff(2,l));
            output_data_struct.MTML.joint2 = collect_mtm_one_joint(config_joint2, mtml_arm, behavior_buff(1,l), behavior_buff(2,l));
            output_data_struct.MTML.joint1 = collect_mtm_one_joint(config_joint1, mtml_arm, behavior_buff(1,l), behavior_buff(2,l));
            mtml_arm.move_joint([0,0,0,0,0,0,0]);
            
        end
        if strcmp(arm_name_string, 'MTMR') | strcmp(arm_name_string, 'MTML&MTMR')
            [config_joint1, config_joint2, config_joint3,config_joint4,config_joint5,config_joint6]= setting_dataCollection(arm_name_R,dataCollection_config_str);
%             output_data_struct.MTMR.joint6 = collect_mtm_one_joint(config_joint6, mtmr_arm, behavior_buff(1,l), behavior_buff(2,l));
%             output_data_struct.MTMR.joint5 = collect_mtm_one_joint(config_joint5, mtmr_arm, behavior_buff(1,l), behavior_buff(2,l));
%             output_data_struct.MTMR.joint4 = collect_mtm_one_joint(config_joint4, mtmr_arm, behavior_buff(1,l), behavior_buff(2,l));
%             output_data_struct.MTMR.joint3 = collect_mtm_one_joint(config_joint3, mtmr_arm, behavior_buff(1,l), behavior_buff(2,l));
            output_data_struct.MTMR.joint2 = collect_mtm_one_joint(config_joint2, mtmr_arm, behavior_buff(1,l), behavior_buff(2,l));
            output_data_struct.MTMR.joint1 = collect_mtm_one_joint(config_joint1, mtmr_arm, behavior_buff(1,l), behavior_buff(2,l));
            mtmr_arm.move_joint([0,0,0,0,0,0,0]);
        end
    end
    
    is_dataCollection_finish = true;
end

function [is_dataCollection_finish, is_quit_data_collection] = check_data_save_path(output_data_root_path)
    is_dataCollection_finish = false;
    is_quit_data_collection = false;
    if exist(output_data_root_path)==7
        input_str = '';
        while(~strcmp(input_str,'y') & ~strcmp(input_str,'n'))
            disp(sprintf('Found existing path:[%s], do you want to cover it? [y]/[n]:  ',output_data_root_path));           
            disp(sprintf('[y]: removing the old path data and create a new empty one '));
            input_str = input(sprintf('[n]: further manipulation of folder will execute :  '),'s');
        end
        if input_str == 'y'
            rmdir(output_data_root_path,'s');
            mkdir(output_data_root_path);
            disp(sprintf('Removing old folder %s and creating new one %s',output_data_root_path));
        elseif input_str == 'n'
            input_str = '';
            while(~strcmp(input_str,'m') & ~strcmp(input_str,'s') & ~strcmp(input_str,'q'))
                disp(sprintf('Which action do you want to take?'));
                disp(sprintf('merge and save new data with old data, and use them to implement LSE together. [m]'));
                disp(sprintf('Skip data collection process and implement LSE. [s]'));
                input_str = input(sprintf('quit GC process. [q]: '),'s');
            end
            if input_str == 'q'
                disp(sprintf('quiting data collection process..'));
                is_dataCollection_finish = false;
                is_quit_data_collection = true;
                return 
            elseif input_str == 's'
                disp(sprintf('Skiping data collection process, Identify paramters with old data'));
                is_dataCollection_finish = true;
                is_quit_data_collection = true;
                return 
            end
        end
    else
        disp(sprintf('Creating Folder %s',output_data_root_path));
        mkdir(output_data_root_path);
    end
end

function argument_checking(GC_ARM, dataCollection_config_str)
    if~(strcmp(GC_ARM,'MTML') | strcmp(GC_ARM,'MTMR') | strcmp(GC_ARM,'MTML&MTMR'))
        error(sprintf(['Input of argument ''GC_ARM''= %s is error, you should input one of the string',...
                       '[''MTML'',''MTMR'',''MTML&MTMR'']'],GC_ARM));
    end
    if exist(dataCollection_config_str) ~= 2
        error(sprintf('Cannot find the json file %s',input_json_str))
    end

    if(~strcmp(dataCollection_config_str(end-4:end),'.json'))
        error(sprintf('file %s is not .json extend',input_json_str))
    end
end