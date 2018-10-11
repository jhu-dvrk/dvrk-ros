function [Regressor_Matrix, Parameter_matrix, bool_Regressor_Mat] = symbolic_gc_dynamic()
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05
%  Reference: This is a modified version of WPI Code
%               link1:https://github.com/WPI-AIM/dvrk_gravity_comp/blob/master/Final_Submission/MATLAB%20code/Regressor_and_Parameter_matrix/Torque_Script.m
%               link2:https://github.com/WPI-AIM/dvrk_gravity_comp/blob/master/Final_Submission/MATLAB%20code/Regressor_and_Parameter_matrix/Matrices_formation_script.m

syms g real;
syms m1 m2 m3 m4 m5 m6  real;
syms cm1_x cm2_x cm3_x cm4_x cm5_x cm6_x real;
syms cm1_y cm2_y cm3_y cm4_y cm5_y cm6_y real;
syms cm1_z cm2_z cm3_z cm4_z cm5_z cm6_z  real;
syms L1 L2 L3 L4_y0 L4_z0 L5_y0 L5_z0 L6_z0 L6_x0 real;
syms q1 q2 q3 q4 q5 q6 real;
syms m3_parallel L3_parallel real;
syms a6_1 a6_2 a6_3 a6_4 drift6;
syms a5_1 a5_2 a5_3 a5_4 drift5;
syms a4_1 a4_2 a4_3 a4_4 drift4;
syms a3_1 a3_2 a3_3 a3_4 drift3;
syms a2_1 a2_2 a2_3 a2_4 drift2;
syms a1_1 a1_2 a1_3 a1_4 drift1;

Torque = [  drift1 + a1_1*q1 + a1_2 *q1^2 + a1_3 *q1^3 + a1_4 *q1^4
   L2*g*m2*sin(q2) + L2*g*m3*sin(q2) + L2*g*m4*sin(q2) + L2*g*m5*sin(q2) + L2*g*m6*sin(q2) + cm2_y*g*m2*cos(q2) + cm2_x*g*m2*sin(q2) + L3*g*m3*cos(q2)*cos(q3) + L3*g*m4*cos(q2)*cos(q3) + L3*g*m5*cos(q2)*cos(q3) + L3*g*m6*cos(q2)*cos(q3) - L4_z0*g*m4*cos(q2)*sin(q3) - L4_z0*g*m4*cos(q3)*sin(q2) - L4_z0*g*m5*cos(q2)*sin(q3) - L4_z0*g*m5*cos(q3)*sin(q2) - L4_z0*g*m6*cos(q2)*sin(q3) - L4_z0*g*m6*cos(q3)*sin(q2) + cm3_x*g*m3*cos(q2)*cos(q3) - L3*g*m3*sin(q2)*sin(q3) - L3*g*m4*sin(q2)*sin(q3) - L3*g*m5*sin(q2)*sin(q3) - L3*g*m6*sin(q2)*sin(q3) - cm3_z*g*m3*cos(q2)*sin(q3) - cm3_z*g*m3*cos(q3)*sin(q2) - cm4_y*g*m4*cos(q2)*sin(q3) - cm4_y*g*m4*cos(q3)*sin(q2) - cm3_x*g*m3*sin(q2)*sin(q3) + cm4_x*g*m4*cos(q2)*cos(q3)*cos(q4) + cm4_z*g*m4*cos(q2)*cos(q3)*sin(q4) - cm5_y*g*m5*cos(q2)*cos(q3)*sin(q4) - cm5_z*g*m5*cos(q2)*cos(q5)*sin(q3) - cm5_z*g*m5*cos(q3)*cos(q5)*sin(q2) - cm6_y*g*m6*cos(q2)*cos(q5)*sin(q3) - cm6_y*g*m6*cos(q3)*cos(q5)*sin(q2) - cm4_x*g*m4*cos(q4)*sin(q2)*sin(q3) - cm5_x*g*m5*cos(q2)*sin(q3)*sin(q5) - cm5_x*g*m5*cos(q3)*sin(q2)*sin(q5) - cm4_z*g*m4*sin(q2)*sin(q3)*sin(q4) + cm5_y*g*m5*sin(q2)*sin(q3)*sin(q4) - cm5_x*g*m5*cos(q4)*cos(q5)*sin(q2)*sin(q3) + cm6_z*g*m6*cos(q2)*cos(q3)*sin(q4)*sin(q6) + cm6_z*g*m6*cos(q2)*cos(q6)*sin(q3)*sin(q5) + cm6_z*g*m6*cos(q3)*cos(q6)*sin(q2)*sin(q5) + cm5_z*g*m5*cos(q4)*sin(q2)*sin(q3)*sin(q5) - cm6_x*g*m6*cos(q6)*sin(q2)*sin(q3)*sin(q4) + cm6_y*g*m6*cos(q4)*sin(q2)*sin(q3)*sin(q5) - cm6_x*g*m6*cos(q2)*sin(q3)*sin(q5)*sin(q6) - cm6_x*g*m6*cos(q3)*sin(q2)*sin(q5)*sin(q6) - cm6_z*g*m6*sin(q2)*sin(q3)*sin(q4)*sin(q6) + cm5_x*g*m5*cos(q2)*cos(q3)*cos(q4)*cos(q5) - cm5_z*g*m5*cos(q2)*cos(q3)*cos(q4)*sin(q5) + cm6_x*g*m6*cos(q2)*cos(q3)*cos(q6)*sin(q4) - cm6_y*g*m6*cos(q2)*cos(q3)*cos(q4)*sin(q5) - cm6_z*g*m6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) + cm6_x*g*m6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) + cm6_z*g*m6*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) - cm6_x*g*m6*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6) + drift2 + a2_1*q2 + a2_2*q2^2 + a2_3*q2^3 + a2_4*q2^4
                                                                                                                                                                                                                                                                                                                                      -m3_parallel*L3_parallel*cos(q2+q3)-(g*(cm4_z*m4*sin(q2 + q3 - q4) - cm4_z*m4*sin(q2 + q3 + q4) - cm4_x*m4*cos(q2 + q3 - q4) - cm4_x*m4*cos(q2 + q3 + q4) - 2*L3*m3*cos(q2 + q3) - 2*L3*m4*cos(q2 + q3) + 2*L4_z0*m4*sin(q2 + q3) - 2*cm3_x*m3*cos(q2 + q3) + 2*cm3_z*m3*sin(q2 + q3) + 2*cm4_y*m4*sin(q2 + q3) - 2*L3*m5*cos(q2)*cos(q3) - 2*L3*m6*cos(q2)*cos(q3) + 2*L4_z0*m5*cos(q2)*sin(q3) + 2*L4_z0*m5*cos(q3)*sin(q2) + 2*L4_z0*m6*cos(q2)*sin(q3) + 2*L4_z0*m6*cos(q3)*sin(q2) + 2*L3*m5*sin(q2)*sin(q3) + 2*L3*m6*sin(q2)*sin(q3) + 2*cm5_y*m5*cos(q2)*cos(q3)*sin(q4) + 2*cm5_z*m5*cos(q2)*cos(q5)*sin(q3) + 2*cm5_z*m5*cos(q3)*cos(q5)*sin(q2) + 2*cm6_y*m6*cos(q2)*cos(q5)*sin(q3) + 2*cm6_y*m6*cos(q3)*cos(q5)*sin(q2) + 2*cm5_x*m5*cos(q2)*sin(q3)*sin(q5) + 2*cm5_x*m5*cos(q3)*sin(q2)*sin(q5) - 2*cm5_y*m5*sin(q2)*sin(q3)*sin(q4) - 2*cm5_x*m5*cos(q2)*cos(q3)*cos(q4)*cos(q5) + 2*cm5_z*m5*cos(q2)*cos(q3)*cos(q4)*sin(q5) - 2*cm6_x*m6*cos(q2)*cos(q3)*cos(q6)*sin(q4) + 2*cm6_y*m6*cos(q2)*cos(q3)*cos(q4)*sin(q5) + 2*cm5_x*m5*cos(q4)*cos(q5)*sin(q2)*sin(q3) - 2*cm6_z*m6*cos(q2)*cos(q3)*sin(q4)*sin(q6) - 2*cm6_z*m6*cos(q2)*cos(q6)*sin(q3)*sin(q5) - 2*cm6_z*m6*cos(q3)*cos(q6)*sin(q2)*sin(q5) - 2*cm5_z*m5*cos(q4)*sin(q2)*sin(q3)*sin(q5) + 2*cm6_x*m6*cos(q6)*sin(q2)*sin(q3)*sin(q4) - 2*cm6_y*m6*cos(q4)*sin(q2)*sin(q3)*sin(q5) + 2*cm6_x*m6*cos(q2)*sin(q3)*sin(q5)*sin(q6) + 2*cm6_x*m6*cos(q3)*sin(q2)*sin(q5)*sin(q6) + 2*cm6_z*m6*sin(q2)*sin(q3)*sin(q4)*sin(q6) + 2*cm6_z*m6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) - 2*cm6_x*m6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) - 2*cm6_z*m6*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) + 2*cm6_x*m6*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6)))/2 + drift3 + a3_1*q3 + a3_2*q3^2 + a3_3*q3^3 + a3_4*q3^4
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                g*sin(q2 + q3)*(cm4_z*m4*cos(q4) - cm5_y*m5*cos(q4) - cm4_x*m4*sin(q4) + cm6_x*m6*cos(q4)*cos(q6) - cm5_x*m5*cos(q5)*sin(q4) + cm6_z*m6*cos(q4)*sin(q6) + cm5_z*m5*sin(q4)*sin(q5) + cm6_y*m6*sin(q4)*sin(q5) + cm6_z*m6*cos(q5)*cos(q6)*sin(q4) - cm6_x*m6*cos(q5)*sin(q4)*sin(q6)) + drift4 + a4_1*q4  + a4_2*q4^2 + a4_3*q4^3 + a4_4*q4^4
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      - g*m6*(cm6_y*cos(q2)*cos(q3)*sin(q5) - cm6_y*sin(q2)*sin(q3)*sin(q5) + cm6_z*cos(q2)*cos(q3)*cos(q5)*cos(q6) + cm6_y*cos(q2)*cos(q4)*cos(q5)*sin(q3) + cm6_y*cos(q3)*cos(q4)*cos(q5)*sin(q2) - cm6_x*cos(q2)*cos(q3)*cos(q5)*sin(q6) - cm6_z*cos(q5)*cos(q6)*sin(q2)*sin(q3) + cm6_x*cos(q5)*sin(q2)*sin(q3)*sin(q6) - cm6_z*cos(q2)*cos(q4)*cos(q6)*sin(q3)*sin(q5) - cm6_z*cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q5) + cm6_x*cos(q2)*cos(q4)*sin(q3)*sin(q5)*sin(q6) + cm6_x*cos(q3)*cos(q4)*sin(q2)*sin(q5)*sin(q6)) - g*m5*(cm5_z*cos(q2)*cos(q3)*sin(q5) - cm5_x*cos(q2)*cos(q3)*cos(q5) + cm5_x*cos(q5)*sin(q2)*sin(q3) - cm5_z*sin(q2)*sin(q3)*sin(q5) + cm5_z*cos(q2)*cos(q4)*cos(q5)*sin(q3) + cm5_z*cos(q3)*cos(q4)*cos(q5)*sin(q2) + cm5_x*cos(q2)*cos(q4)*sin(q3)*sin(q5) + cm5_x*cos(q3)*cos(q4)*sin(q2)*sin(q5))+ drift5 + a5_1*q5 + a5_2*q5^2 + a5_3*q5^3 + a5_4*q5^4
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                g*m6*(cm6_x*cos(q2)*cos(q3)*cos(q6)*sin(q5) + cm6_z*cos(q2)*cos(q6)*sin(q3)*sin(q4) + cm6_z*cos(q3)*cos(q6)*sin(q2)*sin(q4) + cm6_z*cos(q2)*cos(q3)*sin(q5)*sin(q6) - cm6_x*cos(q2)*sin(q3)*sin(q4)*sin(q6) - cm6_x*cos(q3)*sin(q2)*sin(q4)*sin(q6) - cm6_x*cos(q6)*sin(q2)*sin(q3)*sin(q5) - cm6_z*sin(q2)*sin(q3)*sin(q5)*sin(q6) + cm6_z*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) + cm6_z*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6) + cm6_x*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) + cm6_x*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2))+ drift6 + a6_1*q6 + a6_2*q6^2 + a6_3*q6^3 + a6_4*q6^4 
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   0];
%%
% First of all Populate the parametetric matrix
Parameter_matrix(1,1) = L2*m2+L2*m3+L2*m4+L2*m5+L2*m6+cm2_x*m2 ;
m_coeffs(1,1) = L2; m_coeffs(1,2) = m2;
Parameter_matrix(2,1) = cm2_y*m2;
m_coeffs(2,1) = cm2_y; m_coeffs(2,2) = m2;
Parameter_matrix(3,1) = L3*m3+L3*m4+L3*m5+L3*m6+cm3_x*m3;
m_coeffs(3,1) = L3;   m_coeffs(3,2) = m3;
Parameter_matrix(4,1) = cm4_y*m4 +cm3_z*m3 +L4_z0*m4 +L4_z0*m5 +L4_z0*m6 ;
m_coeffs(4,1) =cm4_y; m_coeffs(4,2) = m4;
Parameter_matrix(5,1) = cm4_x*m4;
m_coeffs(5,1) =cm4_x; m_coeffs(5,2) = m4;
Parameter_matrix(6,1) = -cm4_z*m4 + cm5_y*m5;
m_coeffs(6,1) =cm5_y; m_coeffs(6,2) = m5;
Parameter_matrix(7,1) = cm5_z*m5 +cm6_y*m6;
m_coeffs(7,1) =cm5_z; m_coeffs(7,2) = m5;
Parameter_matrix(8,1) = cm5_x*m5;
m_coeffs(8,1) =cm5_x; m_coeffs(8,2) = m5;
Parameter_matrix(9,1) = cm6_z*m6;
m_coeffs(9,1) =cm6_z; m_coeffs(9,2) = m6;
Parameter_matrix(10,1) = cm6_x*m6;
m_coeffs(10,1) =cm6_x; m_coeffs(10,2) = m6;
Parameter_matrix(11,1) = drift1;
m_coeffs(11,1) = drift1; m_coeffs(11,2) = 1;
Parameter_matrix(12,1) = a1_1;
m_coeffs(12,1) = a1_1; m_coeffs(12,2) = 1;
Parameter_matrix(13,1) = a1_2;
m_coeffs(13,1) = a1_2; m_coeffs(13,2) =1;
Parameter_matrix(14,1) = a1_3;
m_coeffs(14,1) = a1_3; m_coeffs(14,2) = 1;
Parameter_matrix(15,1) = a1_4;
m_coeffs(15,1) = a1_4; m_coeffs(15,2) =1;
Parameter_matrix(16,1) = drift2;
m_coeffs(16,1) = drift2; m_coeffs(16,2) = 1;
Parameter_matrix(17,1) = a2_1;
m_coeffs(17,1) = a2_1; m_coeffs(17,2) = 1;
Parameter_matrix(18,1) = a2_2;
m_coeffs(18,1) = a2_2; m_coeffs(18,2) =1;
Parameter_matrix(19,1) = a2_3;
m_coeffs(19,1) = a2_3; m_coeffs(19,2) = 1;
Parameter_matrix(20,1) = a2_4;
m_coeffs(20,1) = a2_4; m_coeffs(20,2) = 1;
Parameter_matrix(21,1) = m3_parallel*L3_parallel;
m_coeffs(21,1) = m3_parallel; m_coeffs(21,2) = L3_parallel;
Parameter_matrix(22,1) = drift3;
m_coeffs(22,1) = drift3; m_coeffs(22,2) = 1;
Parameter_matrix(23,1) = a3_1;
m_coeffs(23,1) = a3_1; m_coeffs(23,2) = 1;
Parameter_matrix(24,1) = a3_2;
m_coeffs(24,1) = a3_2; m_coeffs(24,2) = 1;
Parameter_matrix(25,1) = a3_3;
m_coeffs(25,1) = a3_3; m_coeffs(25,2) = 1;
Parameter_matrix(26,1) = a3_4;
m_coeffs(26,1) = a3_4; m_coeffs(26,2) = 1;
Parameter_matrix(27,1) = drift4;
m_coeffs(27,1) = drift4; m_coeffs(27,2) = 1;
Parameter_matrix(28,1) = a4_1;
m_coeffs(28,1) = a4_1; m_coeffs(28,2) = 1;
Parameter_matrix(29,1) = a4_2;
m_coeffs(29,1) = a4_2; m_coeffs(29,2) = 1;
Parameter_matrix(30,1) = a4_3;
m_coeffs(30,1) = a4_3; m_coeffs(30,2) = 1;
Parameter_matrix(31,1) = a4_4;
m_coeffs(31,1) = a4_4; m_coeffs(31,2) = 1;
Parameter_matrix(32,1) = drift5;
m_coeffs(32,1) = drift5; m_coeffs(32,2) = 1;
Parameter_matrix(33,1) = a5_1;
m_coeffs(33,1) = a5_1; m_coeffs(33,2) = 1;
Parameter_matrix(34,1) = a5_2;
m_coeffs(34,1) = a5_2; m_coeffs(34,2) = 1;
Parameter_matrix(35,1) = a5_3;
m_coeffs(35,1) = a5_3; m_coeffs(35,2) = 1;
Parameter_matrix(36,1) = a5_4;
m_coeffs(36,1) = a5_4; m_coeffs(36,2) = 1;
Parameter_matrix(37,1) = drift6;
m_coeffs(37,1) = drift6; m_coeffs(37,2) = 1;
Parameter_matrix(38,1) = a6_1;
m_coeffs(38,1) = a6_1; m_coeffs(38,2) = 1;
Parameter_matrix(39,1) = a6_2;
m_coeffs(39,1) = a6_2; m_coeffs(39,2) = 1;
Parameter_matrix(40,1) = a6_3;
m_coeffs(40,1) = a6_3; m_coeffs(40,2) = 1;
Parameter_matrix(41,1) = a6_4;
m_coeffs(41,1) = a6_4; m_coeffs(41,2) = 1;

d_size = size(Parameter_matrix,1);
param_num = d_size;
Regressor_Matrix = sym(zeros(7,param_num));

bool_Regressor_Mat = zeros(7,param_num);
bool_Regressor_Mat(2,1:2) = 1;
bool_Regressor_Mat(2:3,3:4) = 1;
bool_Regressor_Mat(2:4,5:6) = 1;
bool_Regressor_Mat(2:5,7:8) = 1;
bool_Regressor_Mat(2:6,9:10) = 1;
bool_Regressor_Mat(1,11:15) = 1;
bool_Regressor_Mat(2,16:20) = 1;
bool_Regressor_Mat(3,21:26) = 1;
bool_Regressor_Mat(4,27:31) = 1;
bool_Regressor_Mat(5,32:36) = 1;
bool_Regressor_Mat(6,37:41) = 1;


%%
% 
for i=1:7
    for j=1:param_num
        if(bool_Regressor_Mat(i,j) == 1)
            if(m_coeffs(j,1) == 1)
                 Regressor_Matrix (i,j)  = Get_One_Cof(Torque(i),m_coeffs(j,2));
            elseif(m_coeffs(j,2) == 1)
                 Regressor_Matrix (i,j)  = Get_One_Cof(Torque(i),m_coeffs(j,1));
            else
                Regressor_Matrix (i,j)  = Get_Cof(Torque(i),m_coeffs(j,2),m_coeffs(j,1));
            end
        else
            Regressor_Matrix (i,j)  = 0;
        end
    end
end

end
%%
% Row 3




%Regressor_Matrix

%T = Regressor_Matrix*Parameter_matrix;

%Diff  = simplify(T - Torque);

















