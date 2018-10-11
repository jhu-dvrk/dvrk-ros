function [MTML_pos_file,MTML_neg_file,MTMR_pos_file,MTMR_neg_file]= setting_GC()
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05

    root_path = '../Data_Collection/CUHK_test';

    MTML_pos_file = strcat(root_path,'/MTML/Train_Joint1/data_pos/4POL/param.mat');
    MTML_neg_file = strcat(root_path,'/MTML/Train_Joint1/data_neg/4POL/param.mat');
    MTMR_pos_file = strcat(root_path,'/MTMR/Train_Joint1/data_pos/4POL/param.mat');
    MTMR_neg_file = strcat(root_path,'/MTMR/Train_Joint1/data_neg/4POL/param.mat');
end