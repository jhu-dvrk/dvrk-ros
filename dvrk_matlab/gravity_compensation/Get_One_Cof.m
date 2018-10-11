function v = Get_One_Cof( T,m)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05
%  Reference: This is a modified version of WPI Code
%               Link: https://github.com/WPI-AIM/dvrk_gravity_comp/blob/master/Final_Submission/MATLAB%20code/Regressor_and_Parameter_matrix/Get_Cof.m

a = coeffs(T,m);
d_size = size(a);
v = a(d_size(2));
end

