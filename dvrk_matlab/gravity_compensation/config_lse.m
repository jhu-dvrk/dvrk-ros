classdef config_lse
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05

   properties
       Joint_No
       std_filter
       symbolic_file
       g_constant
       Is_Plot
       issave_figure
       root_input_path
       fit_method
       input_pos_data_path
       input_neg_data_path
       input_pos_data_files
       input_neg_data_files
       root_output_path
       output_pos_fig_path
       output_neg_fig_path
       output_pos_param_path
       output_neg_param_path
       Output_Param_Joint_No
       prior_param_index
       prior_param_values
   end
   methods
      function obj = config_lse(Joint_No,std_filter,symbolic_file,Is_Plot,...
              issave_figure,root_input_path,fit_method,g_constant,root_output_path)
          if nargin >= 6
               obj.Joint_No =Joint_No;
               obj.std_filter = std_filter;
               obj.symbolic_file = symbolic_file;
               obj.Is_Plot = Is_Plot;
               obj.issave_figure = issave_figure;
               obj.root_input_path = root_input_path;
               obj.input_pos_data_path = strcat(obj.root_input_path,'/data_pos');
               obj.input_neg_data_path = strcat(obj.root_input_path,'/data_neg');
               obj.input_pos_data_files = strcat(obj.input_pos_data_path,'/*mat');
               obj.input_neg_data_files = strcat(obj.input_neg_data_path,'/*mat');
          else
              Error('Number of Argument should be above 4');
          end
          if ~exist('g_constant')
              obj.g_constant = 9.81
          else 
              obj.g_constant = g_constant;
          end
          obj.prior_param_index = [0];
          obj.prior_param_values = [0];
          if ~exist('fit_method')
              obj.fit_method = '4POL';  
          else
              fit_method_list = ['orgin','drift','1POL','2POL','3POL','4POL'];
              if ismember(fit_method, fit_method_list)  
                obj.fit_method = fit_method;
              else
                  error(strcat('please choose one of [', fit_method_list,']'));
              end
          end
          
          if ~exist('root_output_path')
              obj.root_output_path = obj.root_input_path;
          else
              obj.root_output_path = root_output_path;
          end
          obj.output_pos_param_path = [obj.root_output_path,'/data_pos/',obj.fit_method];
          obj.output_neg_param_path = [obj.root_output_path, '/data_neg/',obj.fit_method];
          obj.output_pos_fig_path = [obj.root_output_path, '/data_pos/',obj.fit_method,'/figures'];
          obj.output_neg_fig_path = [obj.root_output_path, '/data_neg/',obj.fit_method,'/figures'];
          obj.Output_Param_Joint_No = [];
          for k=obj.Joint_No:7
              obj.Output_Param_Joint_No(end+1) = k;
          end
          if Joint_No==3
               obj.Output_Param_Joint_No = [2,3,4,5,6,7];
          end
          obj = set_prior(obj);
      end
      function obj = set_prior(obj)
         fit_method_list = {'4POL','3POL','2POL','1POL','drift','origin'};
         joint6_prior_index ={{0},{0,41},{0,40,41},{0,39,40,41},{0,38,39,40,41},{0,37,38,39,40,41}};
         joint5_prior_index ={{0},{0,36},{0,35,36},{0,34,35,36},{0,33,34,35,36},{0,32,33,34,35,36}};
         joint4_prior_index ={{0},{0,31},{0,30,31},{0,29,30,31},{0,28,29,30,31},{0,27,28,29,30,31}};
         joint2_3_prior_index ={{0},{0,20,26},{0,19,20,25,26},{0,18,19,20,24,25,26},...
                                {0,17,18,19,20,23,24,25,26},{0,16,17,18,19,20,22,23,24,25,26}};
         joint1_prior_index ={{0},{0,15},{0,14,15},{0,13,14,15},{0,12,13,14,15},{0,11,12,13,14,15}};
         param_values = {{0},{0,0},{0,0,0},{0,0,0,0},{0,0,0,0,0},{0,0,0,0,0,0}};
         joint2_3_param_values = {{0},{0,0,0},{0,0,0,0,0},...
             {0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0}};
         joint1_index_map = containers.Map(fit_method_list,joint1_prior_index); 
         joint2_3_index_map = containers.Map(fit_method_list,joint2_3_prior_index); 
         joint4_index_map = containers.Map(fit_method_list,joint4_prior_index); 
         joint5_index_map = containers.Map(fit_method_list,joint5_prior_index); 
         joint6_index_map = containers.Map(fit_method_list,joint6_prior_index);
         joint_values_map = containers.Map(fit_method_list,param_values);
         joint2_3_values_map = containers.Map(fit_method_list,joint2_3_param_values);
         if obj.Joint_No == 6
             obj.prior_param_index = joint6_index_map(obj.fit_method);
             obj.prior_param_values = joint_values_map(obj.fit_method);
         end
         if obj.Joint_No == 5
             obj.prior_param_index = joint5_index_map(obj.fit_method);
             obj.prior_param_values = joint_values_map(obj.fit_method);
         end
         if obj.Joint_No == 4
             obj.prior_param_index = joint4_index_map(obj.fit_method);
             obj.prior_param_values = joint_values_map(obj.fit_method);
         end
         if obj.Joint_No == 3 | obj.Joint_No == 2
             obj.prior_param_index = joint2_3_index_map(obj.fit_method);
             obj.prior_param_values = joint2_3_values_map(obj.fit_method);
         end
         if obj.Joint_No == 1
             obj.prior_param_index = joint1_index_map(obj.fit_method);
             obj.prior_param_values = joint_values_map(obj.fit_method);
         end
      end
   end
end