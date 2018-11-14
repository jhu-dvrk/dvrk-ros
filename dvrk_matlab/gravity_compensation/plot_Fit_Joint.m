function plot_Fit_Joint(Torques_data,...
                        Regressor_Matrix,...
                        dynamic_parameters,...
                        Joint_No,... 
                        theta,...
                        g,...
                        isplot,...
                        issave,...
                        save_path,...
                        save_file_index)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05

syms_R_row = Regressor_Matrix(Joint_No,:);
symvar_row = symvar(Regressor_Matrix(Joint_No,:));
for i=1:size(Torques_data,3)
    var_sym2value_map = containers.Map({'q1','q2','q3','q4','q5','q6','q7','g'},...
                                        num2cell([Torques_data(1:7,1,i).',g]));
    var_list = [];
    for k=1:size(symvar_row,2)
        var_list(end+1) = var_sym2value_map(char(symvar_row(k)));
    end
    E = double(subs(syms_R_row,symvar_row,var_list));
    
    F = E*dynamic_parameters;
    
    x(i) = Torques_data(Joint_No,1,i);
    y1(i) = Torques_data(Joint_No,2,i);
    y2(i) = F;
end
x= x.';
x=x*180/pi;
y1= y1.';  
y2= y2.';

if isplot
    figure;
else
    figure('visible', 'off')
end
title_string = sprintf('Actual and Predicted Torque of Joint%d at theta=%d', Joint_No, theta);
xlabel_string = sprintf('Joint %d Angle',Joint_No);
ylabel_string = sprintf('Joint %d Torque',Joint_No);
scatter(x,y1,100);
hold on
plot(x,y2)
title(title_string);
xlabel(xlabel_string);
ylabel(ylabel_string);

if issave == 1
    if exist(save_path)~=7
        mkdir(save_path)
    end
    saveas(gcf, strcat(save_path,'/Figure_',int2str(save_file_index),'_',title_string,'.png'));
    disp(sprintf(strcat('Figure, [',title_string,'.png], has saved.')));
end
end

