function dynamic_param = LSE_MTM_one_joint(config_lse_joint, previous_config)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05

    if ~exist('previous_config')
        disp(sprintf('LSE of Joint%d  has start..',config_lse_joint.Joint_No));
        
        dynamic_param = LSE_Model(config_lse_joint);

     
    else
        disp(sprintf('LSE of Joint%d has start..',config_lse_joint.Joint_No));
        
        % if there is 'previous_config', we pass the path to the result of previous step of LSE to LSE_Model
        dynamic_param = LSE_Model(config_lse_joint,...
                                  previous_config.output_param_path); 
    end  
end  

