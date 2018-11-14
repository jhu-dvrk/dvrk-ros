classdef config_collecting
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05

   properties
      root_data_path
      pos_data_path
      neg_data_path
      init_joint_range
      pos_joint_range
      neg_joint_range
      pos_joint_range_list 
      neg_joint_range_list 
      is_path_planning
      is_collecting_data
      is_neg_dir
      is_pos_dir
      arm_name 
      Theta_Joint_No
      Train_Joint_No
      sample_num
      data_size
      repeat_times
      steady_time
      
   end
   methods
      function obj = config_collecting(Theta_Joint_No,...
                                       theta_angle_list,...
                                       Train_Joint_No,...
                                       train_angle_list,...
                                       arm_name,...
                                       root_data_path,...
                                       sample_num,...
                                       init_joint_range,...
                                       is_path_planning,...
                                       is_collecting_data,...
                                       is_pos_dir,...
                                       is_neg_dir,...
                                       steady_time,...
                                       repeat_times)
          if nargin >= 12
            obj.Theta_Joint_No = Theta_Joint_No;
            obj.Train_Joint_No = Train_Joint_No;
            obj.init_joint_range = num2cell(init_joint_range.'); 
            obj.arm_name = arm_name;
            obj.root_data_path = root_data_path;
            obj.pos_joint_range = obj.init_joint_range; 
            obj.sample_num = sample_num;
            obj.is_path_planning = is_path_planning;
            obj.is_collecting_data = is_collecting_data;
            obj.is_pos_dir = is_pos_dir;
            obj.is_neg_dir = is_neg_dir;
            obj.steady_time = steady_time;
            if ~exist('repeat_times')
              obj.repeat_times = 1;
          else
               obj.repeat_times = repeat_times;
          end
            if ischar(root_data_path)
                obj.root_data_path = root_data_path;
                obj.pos_data_path = strcat(root_data_path,'/',arm_name,'/Train_Joint',...
                    num2str(Train_Joint_No),'/data_pos');
                obj.neg_data_path = strcat(root_data_path,'/',arm_name,'/Train_Joint',...
                    num2str(Train_Joint_No),'/data_neg');
            else
                Error('root data path is not string');
            end
            
            if obj.Train_Joint_No ==1 | obj.Train_Joint_No ==2
                list = zeros(1,obj.repeat_times); 
                list(:) = theta_angle_list;
                obj.pos_joint_range{Theta_Joint_No} = list;
                obj.pos_joint_range{Train_Joint_No} = train_angle_list;
                obj.neg_joint_range = obj.pos_joint_range;
                obj.neg_joint_range{Theta_Joint_No} = list;
                obj.neg_joint_range{Train_Joint_No} = fliplr(train_angle_list);
                obj.pos_joint_range_list = {};
                obj.neg_joint_range_list = {};
                for i= 1:size(list,2)
                    temp = obj.pos_joint_range;
                    temp{Theta_Joint_No} = list(i);
                    obj.pos_joint_range_list{end+1} = temp;
                    temp = obj.neg_joint_range;
                    temp{Theta_Joint_No} = list(i);
                    obj.neg_joint_range_list{end+1} = temp;
                end
            else
                obj.pos_joint_range{Train_Joint_No} = train_angle_list;
                obj.pos_joint_range{Theta_Joint_No} = theta_angle_list;
                obj.neg_joint_range = obj.pos_joint_range;
                obj.neg_joint_range{Train_Joint_No} = fliplr(train_angle_list);
                obj.pos_joint_range_list = {};
                obj.neg_joint_range_list = {};
                for i= 1:size(theta_angle_list,2)
                    temp = obj.pos_joint_range;
                    temp{Theta_Joint_No} = theta_angle_list(i);
                    obj.pos_joint_range_list{end+1} = temp;
                    temp = obj.neg_joint_range;
                    temp{Theta_Joint_No} = theta_angle_list(i);
                    obj.neg_joint_range_list{end+1} = temp;
                end
            end       
            obj.data_size = size(train_angle_list,2) * size(theta_angle_list, 2);    
          else
              Error('Number of Argument should be above 12');
          end
      end
      
      function obj=set_couple_upper_limit(obj, upper_limit)
          % joint(Theta_Joint_No) + joint(Trained_Joint_No) =< upper_limit
          sum = 0;
          pos_joint_range_list =  {};
          neg_joint_range_list =  {};
          for i = 1:size(obj.pos_joint_range_list,2)
              choose_index = obj.pos_joint_range_list{i}{obj.Train_Joint_No} <= upper_limit...
                  -obj.pos_joint_range_list{i}{obj.Theta_Joint_No};
              range_tempt = obj.pos_joint_range_list{i};
              range_tempt{obj.Train_Joint_No} = range_tempt{obj.Train_Joint_No}(choose_index);
              if size(range_tempt{obj.Train_Joint_No},2)~=0
                pos_joint_range_list{end+1} = range_tempt;
              end

              choose_index = obj.neg_joint_range_list{i}{obj.Train_Joint_No} <= upper_limit...
                  -obj.neg_joint_range_list{i}{obj.Theta_Joint_No};
              range_tempt = obj.neg_joint_range_list{i};
              range_tempt{obj.Train_Joint_No} = range_tempt{obj.Train_Joint_No}(choose_index);
              if size(range_tempt{obj.Train_Joint_No},2)~=0
                neg_joint_range_list{end+1} = range_tempt;
              end
              sum = sum + size( range_tempt{obj.Train_Joint_No},2);
          end
          obj.data_size = sum;
          obj.pos_joint_range_list = pos_joint_range_list;
          obj.neg_joint_range_list = neg_joint_range_list;
      end
    function obj=set_couple_lower_limit(obj, lower_limit)
          % joint(Theta_Joint_No) + joint(Trained_Joint_No) =< upper_limit
          sum = 0;
          pos_joint_range_list =  {};
          neg_joint_range_list =  {};
          for i = 1:size(obj.pos_joint_range_list,2)
              choose_index = obj.pos_joint_range_list{i}{obj.Train_Joint_No} >= lower_limit...
                  -obj.pos_joint_range_list{i}{obj.Theta_Joint_No};
              range_tempt = obj.pos_joint_range_list{i};
              range_tempt{obj.Train_Joint_No} = range_tempt{obj.Train_Joint_No}(choose_index);
              if size(range_tempt{obj.Train_Joint_No},2)~=0
                pos_joint_range_list{end+1} = range_tempt;
              end

              choose_index = obj.neg_joint_range_list{i}{obj.Train_Joint_No} >= lower_limit...
                  -obj.neg_joint_range_list{i}{obj.Theta_Joint_No};
              range_tempt = obj.neg_joint_range_list{i};
              range_tempt{obj.Train_Joint_No} = range_tempt{obj.Train_Joint_No}(choose_index);
              if size(range_tempt{obj.Train_Joint_No},2)~=0
                neg_joint_range_list{end+1} = range_tempt;
              end
              sum = sum + size( range_tempt{obj.Train_Joint_No},2);
          end
          obj.data_size = sum;
          obj.pos_joint_range_list = pos_joint_range_list;
          obj.neg_joint_range_list = neg_joint_range_list;
      end      
   end
end