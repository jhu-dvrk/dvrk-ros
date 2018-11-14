classdef config_lse
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05

   properties
       Joint_No
       std_filter
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
       output_param_path
       Output_Param_Joint_No
       prior_param_index
       prior_param_values
   end
   methods
      function obj = config_lse(Joint_No,std_filter,Is_Plot,...
              issave_figure,root_input_path,fit_method,g_constant,root_output_path)
          if nargin >= 6
               obj.Joint_No =Joint_No;
               obj.std_filter = std_filter;
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
          obj.output_param_path = [obj.root_output_path,'/Estimated_Param/',obj.fit_method];
          obj.output_pos_fig_path = [obj.root_output_path,'/Estimated_Param/',obj.fit_method,'/figures/data_pos'];
          obj.output_neg_fig_path = [obj.root_output_path,'/Estimated_Param/',obj.fit_method,'/figures/data_neg'];
          obj.Output_Param_Joint_No = [];
          for k=obj.Joint_No:7
              obj.Output_Param_Joint_No(end+1) = k;
          end
          obj = set_fit_method(obj);
      end
      function obj = set_fit_method(obj)
         % Goal: Choose different fitting method by setting specified dynamic param to zeros
         % For example:
         %         full equation: G(q)+a0+a1*q+a2*q^2+a3*q^3+a4*q^4
         %         3POL ----- G(q)+a0+a1*q+a2*q^2+a3*q^3
         %         then a4 set to zeros
         % Set fitting model
         % fit_method     Dynamic Equation
         % 4POL ----- G(q)+a0+a1*q+a2*q^2+a3*q^3+a4*q^4
         % 3POL ----- G(q)+a0+a1*q+a2*q^2+a3*q^3
         % 2POL ----- G(q)+a0+a1*q+a2*q^2
         % 1POL ----- G(q)+a0+a1*q
         % drift -----G(q)+a0
         % origin ----G(q)
         %Output:
         %param(param_index) = 0;
         %prior_param_index = {[0,[param_index]]};
         
         fit_method_list = {'4POL','3POL','2POL','1POL','drift','origin'};
         joint_prior_index_list = {};
         for i=1:6
             gc_param_num = 10;
             joint_prior_index = {{0},...
                              num2cell([0,i*5+gc_param_num, i*5+gc_param_num+30]),...
                              num2cell([0,i*5+gc_param_num-1:i*5+gc_param_num, i*5+gc_param_num+30-1:i*5+gc_param_num+30]),...
                              num2cell([0,i*5+gc_param_num-2:i*5+gc_param_num, i*5+gc_param_num+30-2:i*5+gc_param_num+30]),...
                              num2cell([0,i*5+gc_param_num-3:i*5+gc_param_num, i*5+gc_param_num+30-3:i*5+gc_param_num+30]),...
                              num2cell([0,i*5+gc_param_num-4:i*5+gc_param_num, i*5+gc_param_num+30-4:i*5+gc_param_num+30])};
             joint_prior_index_list{end+1} = joint_prior_index;
         end
         joint1_prior_index = joint_prior_index_list{1};
         joint2_prior_index = joint_prior_index_list{2};
         joint3_prior_index = joint_prior_index_list{3};
         joint4_prior_index = joint_prior_index_list{4};
         joint5_prior_index = joint_prior_index_list{5};
         joint6_prior_index = joint_prior_index_list{6};
         
         param_values = {num2cell(zeros(1,1)),...
                         num2cell(zeros(1,3)),...
                         num2cell(zeros(1,5)),...
                         num2cell(zeros(1,7)),...
                         num2cell(zeros(1,9)),...
                         num2cell(zeros(1,11))};

         joint1_index_map = containers.Map(fit_method_list,joint1_prior_index); 
         joint2_index_map = containers.Map(fit_method_list,joint2_prior_index); 
         joint3_index_map = containers.Map(fit_method_list,joint3_prior_index); 
         joint4_index_map = containers.Map(fit_method_list,joint4_prior_index); 
         joint5_index_map = containers.Map(fit_method_list,joint5_prior_index); 
         joint6_index_map = containers.Map(fit_method_list,joint6_prior_index);
         
         joint_values_map = containers.Map(fit_method_list,param_values);
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
         if obj.Joint_No == 3 
             obj.prior_param_index = joint3_index_map(obj.fit_method);
             obj.prior_param_values = joint_values_map(obj.fit_method);
         end
          if obj.Joint_No == 2
             obj.prior_param_index = joint2_index_map(obj.fit_method);
             obj.prior_param_values = joint_values_map(obj.fit_method);
         end
         if obj.Joint_No == 1
             obj.prior_param_index = joint1_index_map(obj.fit_method);
             obj.prior_param_values = joint_values_map(obj.fit_method);
         end
      end
   end
end