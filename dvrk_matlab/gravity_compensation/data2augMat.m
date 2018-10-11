function  [R2_augmented, T2_augmented] = data2augMat(Torques_data, sym_R_mat,g)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05

%% Plug Training Data to Augumented Regressor Matrix and Augumented Torque Matrix
qs = [symvar(sym_R_mat)];
% number of data sets
s= size(Torques_data); size_R = size(sym_R_mat);
param_num = size_R(2); number_of_data_sets = s(3);

R2_augmented = zeros(number_of_data_sets*7,param_num);
T2_augmented = zeros(number_of_data_sets*7,1);

q_sub = zeros(7,1); tor_sub = zeros(7,1);
for i=1:number_of_data_sets
    
    % load angles values
    q_sub = Torques_data(:,1,i);
    
    % load torques
    tor_sub = Torques_data(1:7,2,i);
    
   
    A = double(subs(sym_R_mat,qs,[g q_sub(1:6).']));
    
    if (i==1)
       R2_augmented(1:7,:) = A; 
       T2_augmented(1:7,:) = tor_sub;
    
    else
        
        R2_augmented(7*i-6:7*i,:) = A;
        T2_augmented(7*i-6:7*i,:) = tor_sub;
    end
end 

end