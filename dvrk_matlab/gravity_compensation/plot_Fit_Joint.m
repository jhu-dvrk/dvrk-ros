function plot_Fit_Joint(Torques_data,Regressor_Matrix, dynamic_parameters,Joint_No,... 
theta, isplot,issave, save_path)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05

%Torques_data = saved_data;
data_set =  size(Torques_data);
for i=1:data_set(3)
    q_sub = Torques_data(:,1,i);
    qs = [symvar(Regressor_Matrix)];
    E = double(subs(Regressor_Matrix,qs,[9.81 q_sub(1:6).']));

    F = E*dynamic_parameters;
    x(i) = Torques_data(Joint_No,1,i);
    y1(i) = Torques_data(Joint_No,2,i);
    y2(i) = F(Joint_No);
end
x= x.';
x=x*180/pi;
y1= y1.';  y2= y2.';

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
    saveas(gcf, strcat(save_path,'/',title_string,'.png'));
    disp(sprintf(strcat('Figure, [',title_string,'.png], has saved.')));
end
end

