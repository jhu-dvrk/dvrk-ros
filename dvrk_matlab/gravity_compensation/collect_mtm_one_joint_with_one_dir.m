function data_file_list = collect_mtm_one_joint_with_one_dir(config,...
                                                              mtm_arm,...
                                                              joint_range_list,...
                                                              data_save_path,...
                                                              is_path_planning,...
                                                              is_collecting_data,...
                                                              dir_name)
                                                          
       arm_name = config.arm_name;
       sample_num = config.sample_num;
       steady_time = config.steady_time;
       data_file_list = {};
       
       if is_path_planning
            disp(sprintf('Path Planning by %s direction now..', dir_name));
            disp(sprintf('If MTM hit the environment, please hit E-Button to stop instantly!')); 
        end
        for  k = 1:size(joint_range_list,2)
            if config.Train_Joint_No ==1
                Theta = 0;
            else
                Theta = joint_range_list{k}{config.Theta_Joint_No};
                Theta = int32(rad2deg(Theta));
            end
            theta_string = sprintf('theta%d-',Theta);
            joint_range = joint_range_list{k};
            [joint_trajectory,jranges_ranges] = generate_joint_grid(joint_range);
            sample_size = size(joint_trajectory,2);
            desired_effort = zeros(7,sample_size, sample_num);
            current_position = zeros(7,sample_size,sample_num);
            
            %Planning the trajectory to pre-plan
            if is_path_planning
                if (k==1 || k==size(joint_range_list,2))
                    mtm_arm.move_joint(joint_trajectory(:,1));
                    mtm_arm.move_joint(joint_trajectory(:,int32((1+end)/2)));
                    mtm_arm.move_joint(joint_trajectory(:,end));
                end
            end
            if is_collecting_data                   
                disp(sprintf('Start to collect Torque data of Theta Joint, Joint%d, with angle %d by %s direction',config.Theta_Joint_No,...
                                                                                                                   Theta,...
                                                                                                                   dir_name));

                for i=1:sample_size
                    mtm_arm.move_joint(joint_trajectory(:,i));
                    pause(steady_time);
                    for j=1:sample_num
                        [position, velocity, desired_effort(:,i,j)] = mtm_arm.get_state_joint_desired();
                        [current_position(:,i,j), velocity, effort] = mtm_arm.get_state_joint_current();
                    end 
                    disp(sprintf('Moving Train Joint, Joint%d, to angle %d',config.Train_Joint_No, int32(rad2deg(joint_trajectory(config.Train_Joint_No,i)))));
                end
                if exist(data_save_path)~=7
                    mkdir(data_save_path)
                end
                file_str = strcat(data_save_path,'/',theta_string,datestr(datetime('now'),...
                    'mmmm-dd-yyyy-HH-MM-SS'),arm_name,'.mat');
                save(file_str,...
                    'joint_trajectory','jranges_ranges','desired_effort',...
                    'current_position','Theta');
                data_file_list{end+1} = file_str;
            end
        end
end
