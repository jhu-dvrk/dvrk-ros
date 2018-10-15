function [GC_controllers,output_data_struct] = gc_demo(GC_ARM)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05

    % Argument Checking
    if~ischar(GC_ARM) 
        error('Both Argument ''GC_ARM'' should be string object');
    end
    if~(strcmp(GC_ARM,'MTML') | strcmp(GC_ARM,'MTMR') | strcmp(GC_ARM,'MTML&MTMR'))
        error(sprintf(['Input of argument ''GC_ARM''= %s is error, you should input one of the string',...
                       '[''MTML'',''MTMR'',''MTML&MTMR'']'],GC_ARM));
    end


    % assign string of config json files
    dataCollection_config_json_str = 'dataCollection_config.json';
    LSE_config_json_str =  'MLSE_config.json';
    GC_controller_config_json_str = 'GC_controller_config.json';

    % Pre Setting Wizard
    dataCollection_config_json_str = wizard_datacollection_config(GC_ARM, dataCollection_config_json_str);
    input_str = '';
    while(~strcmp(input_str,'c') & ~strcmp(input_str,'q'))
        input_str = input(sprintf('Pre Setting Wizard has finish. Do you want to continue[c] following gc process or quit[q] (c/q):  '),'s');
    end
    if input_str == 'q'
        GC_controllers = [];
        output_data_struct = [];
        return
    end

    % execute dataCollection
    [is_dataCollection_finish, output_data_struct] = dataCollection(GC_ARM, dataCollection_config_json_str);
    
    % execute MTM LSE after dataCollection finish
    if is_dataCollection_finish
        [is_MTM_lse_finish, output_dynamic_matrix, output_lse_config] = MLSE(GC_ARM,...
                                                                             LSE_config_json_str,...
                                                                             output_data_struct.output_data_root_path);
    else
        return
    end
    output_data_struct.output_dynamic_matrix = output_dynamic_matrix;
    
    % execute GC_controller after Multi-steps LSE has finished
    if(strcmp(GC_ARM,'MTML') & ~is_MTM_lse_finish.MTML)
        disp(sprintf('Some collecing data for MTML is missing, please check the folder or restart the program.'));
        return
    elseif(strcmp(GC_ARM,'MTMR') & ~is_MTM_lse_finish.MTMR)
        disp(sprintf('Some collecing data for MTMR is missing, please check the folder or restart the program.'));
        return
    elseif(strcmp(GC_ARM,'MTML&MTMR') & (~is_MTM_lse_finish.MTML|~is_MTM_lse_finish.MTMR))
        if(~is_MTM_lse_finish.MTML)
            disp(sprintf('Some collecing data for MTML is missing, please check the folder or restart the program.'));
        elseif(~is_MTM_lse_finish.MTMR)
            disp(sprintf('Some collecing data for MTMR is missing, please check the folder or restart the program.'));
        end
    else
        while(true)
            [is_test_pass,GC_Controllers] = GC_controller(GC_ARM,...
                                           GC_controller_config_json_str,...
                                           output_dynamic_matrix,...
                                           output_lse_config.fit_method,...
                                           output_lse_config.g_constant);
            if is_test_pass
                break
            else
                input_str = '';
                while(~strcmp(input_str,'y') & ~strcmp(input_str,'n'))
                    input_str = input(sprintf('Do you want to restart the gc controller and controller test? y--yes, n--no [y/n]:  '),'s');
                end
                if input_str == 'n'
                   return
                end    
            end
        end
    end
end
