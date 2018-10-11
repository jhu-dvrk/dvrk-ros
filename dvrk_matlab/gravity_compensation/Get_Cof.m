function v = Get_Cof( T,m,L )
%This is a copy of WPI script
%Link: https://github.com/WPI-AIM/dvrk_gravity_comp/blob/master/Final_Submission/MATLAB%20code/Regressor_and_Parameter_matrix/Get_Cof.m
a = coeffs(T,m);
d_size = size(a);
b = coeffs(a(d_size(2)),L);
d_size = size(b);
v = b(d_size(2));
end

