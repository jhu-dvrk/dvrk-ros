function joint_data_list = collect_mtm_torques(config,mtm_arm, is_path_planning, is_collecting_data)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05

    arm_name = config.arm_name;
    sample_num = config.sample_num;
    pos_joint_range_list = config.pos_joint_range_list;
    neg_joint_range_list = config.neg_joint_range_list; 
    joint_data_list.pos = {};
    joint_data_list.neg = {};
    
    % Collect Torque data by positive direction
    if config.is_pos_dir
        if is_path_planning
            disp(sprintf('Path Planning by positive direction now..'));
            disp(sprintf('If MTM hit the environment, please hit E-Button to stop instantly!')); 
        end
        for  k = 1:size(pos_joint_range_list,2)
            if config.Train_Joint_No ==1
                Theta = 0;
            else
                Theta = pos_joint_range_list{k}{config.Theta_Joint_No};
                Theta = int32(rad2deg(Theta));
            end
            theta_string = sprintf('theta%d-',Theta);
            joint_range = pos_joint_range_list{k};
            [joint_trajectory,jranges_ranges] = generate_joint_grid(joint_range);
            sample_size = size(joint_trajectory,2);
            desired_effort = zeros(7,sample_size, sample_num);
            current_position = zeros(7,sample_size,sample_num);
            
            %Planning the trajectory to pre-plan
            if is_path_planning
                if (k==1 || k==size(pos_joint_range_list,2))
                    mtm_arm.move_joint(joint_trajectory(:,1));
                    mtm_arm.move_joint(joint_trajectory(:,int32((1+end)/2)));
                    mtm_arm.move_joint(joint_trajectory(:,end));
                end
            end
            if is_collecting_data                   
                disp(sprintf('Start to collect Torque data of Theta Joint, Joint%d, with angle %d by positive direction',config.Theta_Joint_No,Theta));

                for i=1:sample_size
                    mtm_arm.move_joint(joint_trajectory(:,i));
                    pause(0.2);
                    for j=1:sample_num
                        [position, velocity, desired_effort(:,i,j)] = mtm_arm.get_state_joint_desired();
                        [current_position(:,i,j), velocity, effort] = mtm_arm.get_state_joint_current();
                        %pause(0.1);
                    end 
                    disp(sprintf('Moving Train Joint, Joint%d, to angle %d',config.Train_Joint_No, int32(rad2deg(joint_trajectory(config.Train_Joint_No,i)))));
                end
                if exist(config.pos_data_path)~=7
                    mkdir(config.pos_data_path)
                end
                file_str = strcat(config.pos_data_path,'/',theta_string,datestr(datetime('now'),...
                    'mmmm-dd-yyyy-HH-MM-SS'),arm_name,'.mat');
                save(file_str,...
                    'joint_trajectory','jranges_ranges','desired_effort',...
                    'current_position','Theta');
                joint_data_list.pos{end+1} = file_str;
            end
        end
    end
    
     % Collect Torque data by negative direction
    if config.is_neg_dir
         if is_path_planning
            disp(sprintf('Path Planning by negative direction now..'));
            disp(sprintf('If MTM hit the environment, please hit E-Button to stop instantly!')); 
        end
        for k = 1:size(neg_joint_range_list,2)
            if config.Train_Joint_No ==1
                Theta = 0;
            else
                Theta = neg_joint_range_list{k}{config.Theta_Joint_No};
                Theta = int32(rad2deg(Theta));
            end
           theta_string = sprintf('theta%d-', Theta);
            joint_range = neg_joint_range_list{k};
            [joint_trajectory,jranges_ranges] = generate_joint_grid(joint_range);
            sample_size = size(joint_trajectory,2);
            desired_effort = zeros(7,sample_size, sample_num);
            current_position = zeros(7,sample_size,sample_num);


            if is_collecting_data
                disp(sprintf('Start to collect Torque data of Theta Joint, Joint%d, with angle %d by negative direction',config.Theta_Joint_No, Theta));
                for i=1:sample_size
                    mtm_arm.move_joint(joint_trajectory(:,i));
                    pause(0.2);
                    for j=1:sample_num
                        [position, velocity, desired_effort(:,i,j)] = mtm_arm.get_state_joint_desired();
                        [current_position(:,i,j), velocity, effort] = mtm_arm.get_state_joint_current();
                        %pause(0.1);
                    end    
                    disp(sprintf('Moving Train Joint, Joint%d, to angle %d',config.Train_Joint_No, int32(rad2deg(joint_trajectory(config.Train_Joint_No,i)))));
                end
                if exist(config.neg_data_path)~=7
                    mkdir(config.neg_data_path)
                end
                save(strcat(config.neg_data_path,'/',theta_string,datestr(datetime('now'),...
                    'mmmm-dd-yyyy-HH-MM-SS'),arm_name,'.mat'),...
                    'joint_trajectory','jranges_ranges','desired_effort',...
                    'current_position','Theta');
                joint_data_list.neg{end+1} = file_str;
            end
        end
    end

end

