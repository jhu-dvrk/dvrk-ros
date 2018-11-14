function [Regressor_Matrix_Pos,Regressor_Matrix_Neg, Parameter_matrix, bool_Regressor_Mat] = symbolic_gc_dynamic()
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
syms a6_1_pos a6_2_pos a6_3_pos a6_4_pos drift6_pos;
syms a5_1_pos a5_2_pos a5_3_pos a5_4_pos drift5_pos;
syms a4_1_pos a4_2_pos a4_3_pos a4_4_pos drift4_pos;
syms a3_1_pos a3_2_pos a3_3_pos a3_4_pos drift3_pos;
syms a2_1_pos a2_2_pos a2_3_pos a2_4_pos drift2_pos;
syms a1_1_pos a1_2_pos a1_3_pos a1_4_pos drift1_pos;

syms a6_1_neg a6_2_neg a6_3_neg a6_4_neg drift6_neg;
syms a5_1_neg a5_2_neg a5_3_neg a5_4_neg drift5_neg;
syms a4_1_neg a4_2_neg a4_3_neg a4_4_neg drift4_neg;
syms a3_1_neg a3_2_neg a3_3_neg a3_4_neg drift3_neg;
syms a2_1_neg a2_2_neg a2_3_neg a2_4_neg drift2_neg;
syms a1_1_neg a1_2_neg a1_3_neg a1_4_neg drift1_neg;


Torque = [  0
   L2*g*m2*sin(q2) + L2*g*m3*sin(q2) + L2*g*m4*sin(q2) + L2*g*m5*sin(q2) + L2*g*m6*sin(q2) + cm2_y*g*m2*cos(q2) + cm2_x*g*m2*sin(q2) + L3*g*m3*cos(q2)*cos(q3) + L3*g*m4*cos(q2)*cos(q3) + L3*g*m5*cos(q2)*cos(q3) + L3*g*m6*cos(q2)*cos(q3) - L4_z0*g*m4*cos(q2)*sin(q3) - L4_z0*g*m4*cos(q3)*sin(q2) - L4_z0*g*m5*cos(q2)*sin(q3) - L4_z0*g*m5*cos(q3)*sin(q2) - L4_z0*g*m6*cos(q2)*sin(q3) - L4_z0*g*m6*cos(q3)*sin(q2) + cm3_x*g*m3*cos(q2)*cos(q3) - L3*g*m3*sin(q2)*sin(q3) - L3*g*m4*sin(q2)*sin(q3) - L3*g*m5*sin(q2)*sin(q3) - L3*g*m6*sin(q2)*sin(q3) - cm3_z*g*m3*cos(q2)*sin(q3) - cm3_z*g*m3*cos(q3)*sin(q2) - cm4_y*g*m4*cos(q2)*sin(q3) - cm4_y*g*m4*cos(q3)*sin(q2) - cm3_x*g*m3*sin(q2)*sin(q3) + cm4_x*g*m4*cos(q2)*cos(q3)*cos(q4) + cm4_z*g*m4*cos(q2)*cos(q3)*sin(q4) - cm5_y*g*m5*cos(q2)*cos(q3)*sin(q4) - cm5_z*g*m5*cos(q2)*cos(q5)*sin(q3) - cm5_z*g*m5*cos(q3)*cos(q5)*sin(q2) - cm6_y*g*m6*cos(q2)*cos(q5)*sin(q3) - cm6_y*g*m6*cos(q3)*cos(q5)*sin(q2) - cm4_x*g*m4*cos(q4)*sin(q2)*sin(q3) - cm5_x*g*m5*cos(q2)*sin(q3)*sin(q5) - cm5_x*g*m5*cos(q3)*sin(q2)*sin(q5) - cm4_z*g*m4*sin(q2)*sin(q3)*sin(q4) + cm5_y*g*m5*sin(q2)*sin(q3)*sin(q4) - cm5_x*g*m5*cos(q4)*cos(q5)*sin(q2)*sin(q3) + cm6_z*g*m6*cos(q2)*cos(q3)*sin(q4)*sin(q6) + cm6_z*g*m6*cos(q2)*cos(q6)*sin(q3)*sin(q5) + cm6_z*g*m6*cos(q3)*cos(q6)*sin(q2)*sin(q5) + cm5_z*g*m5*cos(q4)*sin(q2)*sin(q3)*sin(q5) - cm6_x*g*m6*cos(q6)*sin(q2)*sin(q3)*sin(q4) + cm6_y*g*m6*cos(q4)*sin(q2)*sin(q3)*sin(q5) - cm6_x*g*m6*cos(q2)*sin(q3)*sin(q5)*sin(q6) - cm6_x*g*m6*cos(q3)*sin(q2)*sin(q5)*sin(q6) - cm6_z*g*m6*sin(q2)*sin(q3)*sin(q4)*sin(q6) + cm5_x*g*m5*cos(q2)*cos(q3)*cos(q4)*cos(q5) - cm5_z*g*m5*cos(q2)*cos(q3)*cos(q4)*sin(q5) + cm6_x*g*m6*cos(q2)*cos(q3)*cos(q6)*sin(q4) - cm6_y*g*m6*cos(q2)*cos(q3)*cos(q4)*sin(q5) - cm6_z*g*m6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) + cm6_x*g*m6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) + cm6_z*g*m6*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) - cm6_x*g*m6*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6) 
                                                                                                                                                                                                                                                                                                                                      -(g*(cm4_z*m4*sin(q2 + q3 - q4) - cm4_z*m4*sin(q2 + q3 + q4) - cm4_x*m4*cos(q2 + q3 - q4) - cm4_x*m4*cos(q2 + q3 + q4) - 2*L3*m3*cos(q2 + q3) - 2*L3*m4*cos(q2 + q3) + 2*L4_z0*m4*sin(q2 + q3) - 2*cm3_x*m3*cos(q2 + q3) + 2*cm3_z*m3*sin(q2 + q3) + 2*cm4_y*m4*sin(q2 + q3) - 2*L3*m5*cos(q2)*cos(q3) - 2*L3*m6*cos(q2)*cos(q3) + 2*L4_z0*m5*cos(q2)*sin(q3) + 2*L4_z0*m5*cos(q3)*sin(q2) + 2*L4_z0*m6*cos(q2)*sin(q3) + 2*L4_z0*m6*cos(q3)*sin(q2) + 2*L3*m5*sin(q2)*sin(q3) + 2*L3*m6*sin(q2)*sin(q3) + 2*cm5_y*m5*cos(q2)*cos(q3)*sin(q4) + 2*cm5_z*m5*cos(q2)*cos(q5)*sin(q3) + 2*cm5_z*m5*cos(q3)*cos(q5)*sin(q2) + 2*cm6_y*m6*cos(q2)*cos(q5)*sin(q3) + 2*cm6_y*m6*cos(q3)*cos(q5)*sin(q2) + 2*cm5_x*m5*cos(q2)*sin(q3)*sin(q5) + 2*cm5_x*m5*cos(q3)*sin(q2)*sin(q5) - 2*cm5_y*m5*sin(q2)*sin(q3)*sin(q4) - 2*cm5_x*m5*cos(q2)*cos(q3)*cos(q4)*cos(q5) + 2*cm5_z*m5*cos(q2)*cos(q3)*cos(q4)*sin(q5) - 2*cm6_x*m6*cos(q2)*cos(q3)*cos(q6)*sin(q4) + 2*cm6_y*m6*cos(q2)*cos(q3)*cos(q4)*sin(q5) + 2*cm5_x*m5*cos(q4)*cos(q5)*sin(q2)*sin(q3) - 2*cm6_z*m6*cos(q2)*cos(q3)*sin(q4)*sin(q6) - 2*cm6_z*m6*cos(q2)*cos(q6)*sin(q3)*sin(q5) - 2*cm6_z*m6*cos(q3)*cos(q6)*sin(q2)*sin(q5) - 2*cm5_z*m5*cos(q4)*sin(q2)*sin(q3)*sin(q5) + 2*cm6_x*m6*cos(q6)*sin(q2)*sin(q3)*sin(q4) - 2*cm6_y*m6*cos(q4)*sin(q2)*sin(q3)*sin(q5) + 2*cm6_x*m6*cos(q2)*sin(q3)*sin(q5)*sin(q6) + 2*cm6_x*m6*cos(q3)*sin(q2)*sin(q5)*sin(q6) + 2*cm6_z*m6*sin(q2)*sin(q3)*sin(q4)*sin(q6) + 2*cm6_z*m6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) - 2*cm6_x*m6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) - 2*cm6_z*m6*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) + 2*cm6_x*m6*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6)))/2 
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                g*sin(q2 + q3)*(cm4_z*m4*cos(q4) - cm5_y*m5*cos(q4) - cm4_x*m4*sin(q4) + cm6_x*m6*cos(q4)*cos(q6) - cm5_x*m5*cos(q5)*sin(q4) + cm6_z*m6*cos(q4)*sin(q6) + cm5_z*m5*sin(q4)*sin(q5) + cm6_y*m6*sin(q4)*sin(q5) + cm6_z*m6*cos(q5)*cos(q6)*sin(q4) - cm6_x*m6*cos(q5)*sin(q4)*sin(q6)) 
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      - g*m6*(cm6_y*cos(q2)*cos(q3)*sin(q5) - cm6_y*sin(q2)*sin(q3)*sin(q5) + cm6_z*cos(q2)*cos(q3)*cos(q5)*cos(q6) + cm6_y*cos(q2)*cos(q4)*cos(q5)*sin(q3) + cm6_y*cos(q3)*cos(q4)*cos(q5)*sin(q2) - cm6_x*cos(q2)*cos(q3)*cos(q5)*sin(q6) - cm6_z*cos(q5)*cos(q6)*sin(q2)*sin(q3) + cm6_x*cos(q5)*sin(q2)*sin(q3)*sin(q6) - cm6_z*cos(q2)*cos(q4)*cos(q6)*sin(q3)*sin(q5) - cm6_z*cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q5) + cm6_x*cos(q2)*cos(q4)*sin(q3)*sin(q5)*sin(q6) + cm6_x*cos(q3)*cos(q4)*sin(q2)*sin(q5)*sin(q6)) - g*m5*(cm5_z*cos(q2)*cos(q3)*sin(q5) - cm5_x*cos(q2)*cos(q3)*cos(q5) + cm5_x*cos(q5)*sin(q2)*sin(q3) - cm5_z*sin(q2)*sin(q3)*sin(q5) + cm5_z*cos(q2)*cos(q4)*cos(q5)*sin(q3) + cm5_z*cos(q3)*cos(q4)*cos(q5)*sin(q2) + cm5_x*cos(q2)*cos(q4)*sin(q3)*sin(q5) + cm5_x*cos(q3)*cos(q4)*sin(q2)*sin(q5))
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                g*m6*(cm6_x*cos(q2)*cos(q3)*cos(q6)*sin(q5) + cm6_z*cos(q2)*cos(q6)*sin(q3)*sin(q4) + cm6_z*cos(q3)*cos(q6)*sin(q2)*sin(q4) + cm6_z*cos(q2)*cos(q3)*sin(q5)*sin(q6) - cm6_x*cos(q2)*sin(q3)*sin(q4)*sin(q6) - cm6_x*cos(q3)*sin(q2)*sin(q4)*sin(q6) - cm6_x*cos(q6)*sin(q2)*sin(q3)*sin(q5) - cm6_z*sin(q2)*sin(q3)*sin(q5)*sin(q6) + cm6_z*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) + cm6_z*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6) + cm6_x*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) + cm6_x*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2))
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   0];


                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
Torque(1) = Torque(1) + (drift1_pos+drift1_neg) + (a1_1_pos+a1_1_neg)*q1 + (a1_2_pos+a1_2_neg)*q1^2 + (a1_3_pos+a1_3_neg)*q1^3 + (a1_4_pos+a1_4_neg)*q1^4;
Torque(2) = Torque(2) + (drift2_pos+drift2_neg) + (a2_1_pos+a2_1_neg)*q2 + (a2_2_pos+a2_2_neg)*q2^2 + (a2_3_pos+a2_3_neg)*q2^3 + (a2_4_pos+a2_4_neg)*q2^4;
Torque(3) = Torque(3) + (drift3_pos+drift3_neg) + (a3_1_pos+a3_1_neg)*q3 + (a3_2_pos+a3_2_neg)*q3^2 + (a3_3_pos+a3_3_neg)*q3^3 + (a3_4_pos+a3_4_neg)*q3^4;
Torque(4) = Torque(4) + (drift4_pos+drift4_neg) + (a4_1_pos+a4_1_neg)*q4 + (a4_2_pos+a4_2_neg)*q4^2 + (a4_3_pos+a4_3_neg)*q4^3 + (a4_4_pos+a4_4_neg)*q4^4;
Torque(5) = Torque(5) + (drift5_pos+drift5_neg) + (a5_1_pos+a5_1_neg)*q5 + (a5_2_pos+a5_2_neg)*q5^2 + (a5_3_pos+a5_3_neg)*q5^3 + (a5_4_pos+a5_4_neg)*q5^4;
Torque(6) = Torque(6) + (drift6_pos+drift6_neg) + (a6_1_pos+a6_1_neg)*q6 + (a6_2_pos+a6_2_neg)*q6^2 + (a6_3_pos+a6_3_neg)*q6^3 + (a6_4_pos+a6_4_neg)*q6^4;


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

Parameter_matrix(11,1) = drift1_pos;

Parameter_matrix(12,1) = a1_1_pos;

Parameter_matrix(13,1) = a1_2_pos;

Parameter_matrix(14,1) = a1_3_pos;

Parameter_matrix(15,1) = a1_4_pos;

Parameter_matrix(16,1) = drift2_pos;

Parameter_matrix(17,1) = a2_1_pos;

Parameter_matrix(18,1) = a2_2_pos;

Parameter_matrix(19,1) = a2_3_pos;

Parameter_matrix(20,1) = a2_4_pos;

Parameter_matrix(21,1) = drift3_pos;

Parameter_matrix(22,1) = a3_1_pos;

Parameter_matrix(23,1) = a3_2_pos;

Parameter_matrix(24,1) = a3_3_pos;

Parameter_matrix(25,1) = a3_4_pos;

Parameter_matrix(26,1) = drift4_pos;

Parameter_matrix(27,1) = a4_1_pos;

Parameter_matrix(28,1) = a4_2_pos;

Parameter_matrix(29,1) = a4_3_pos;

Parameter_matrix(30,1) = a4_4_pos;

Parameter_matrix(31,1) = drift5_pos;

Parameter_matrix(32,1) = a5_1_pos;

Parameter_matrix(33,1) = a5_2_pos;

Parameter_matrix(34,1) = a5_3_pos;

Parameter_matrix(35,1) = a5_4_pos;

Parameter_matrix(36,1) = drift6_pos;

Parameter_matrix(37,1) = a6_1_pos;

Parameter_matrix(38,1) = a6_2_pos;

Parameter_matrix(39,1) = a6_3_pos;

Parameter_matrix(40,1) = a6_4_pos;

Parameter_matrix(41,1) = drift1_neg;

Parameter_matrix(42,1) = a1_1_neg;

Parameter_matrix(43,1) = a1_2_neg;

Parameter_matrix(44,1) = a1_3_neg;

Parameter_matrix(45,1) = a1_4_neg;

Parameter_matrix(46,1) = drift2_neg;

Parameter_matrix(47,1) = a2_1_neg;

Parameter_matrix(48,1) = a2_2_neg;

Parameter_matrix(49,1) = a2_3_neg;

Parameter_matrix(50,1) = a2_4_neg;

Parameter_matrix(51,1) = drift3_neg;

Parameter_matrix(52,1) = a3_1_neg;

Parameter_matrix(53,1) = a3_2_neg;

Parameter_matrix(54,1) = a3_3_neg;

Parameter_matrix(55,1) = a3_4_neg;

Parameter_matrix(56,1) = drift4_neg;

Parameter_matrix(57,1) = a4_1_neg;

Parameter_matrix(58,1) = a4_2_neg;

Parameter_matrix(59,1) = a4_3_neg;

Parameter_matrix(60,1) = a4_4_neg;

Parameter_matrix(61,1) = drift5_neg;

Parameter_matrix(62,1) = a5_1_neg;

Parameter_matrix(63,1) = a5_2_neg;

Parameter_matrix(64,1) = a5_3_neg;

Parameter_matrix(65,1) = a5_4_neg;

Parameter_matrix(66,1) = drift6_neg;

Parameter_matrix(67,1) = a6_1_neg;

Parameter_matrix(68,1) = a6_2_neg;

Parameter_matrix(69,1) = a6_3_neg;

Parameter_matrix(70,1) = a6_4_neg;

m_coeffs(11:70,1) = Parameter_matrix(11:70);
m_coeffs(11:70,2) = 1;


param_num = size(Parameter_matrix,1);
Regressor_Matrix = sym(zeros(7,param_num));

% bool_Regressor_Mat, if Regressor(i,j)~=Null, bool_Regressor_Mat=1; else bool_Regressor_Mat=0;
bool_Regressor_Mat = zeros(7,param_num);
bool_Regressor_Mat(2,1:2) = 1;
bool_Regressor_Mat(2:3,3:4) = 1;
bool_Regressor_Mat(2:4,5:6) = 1;
bool_Regressor_Mat(2:5,7:8) = 1;
bool_Regressor_Mat(2:6,9:10) = 1;
bool_Regressor_Mat(1,11:15) = 1;
bool_Regressor_Mat(2,16:20) = 1;
bool_Regressor_Mat(3,21:25) = 1;
bool_Regressor_Mat(4,26:30) = 1;
bool_Regressor_Mat(5,31:35) = 1;
bool_Regressor_Mat(6,36:40) = 1;

bool_Regressor_Mat(1,41:45) = 1;
bool_Regressor_Mat(2,46:50) = 1;
bool_Regressor_Mat(3,51:55) = 1;
bool_Regressor_Mat(4,56:60) = 1;
bool_Regressor_Mat(5,61:65) = 1;
bool_Regressor_Mat(6,66:70) = 1;

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

% % Testing if T=Regressor_Matrix*Parameter_matrix
%  T = Regressor_Matrix*Parameter_matrix;
%  Diff  = simplify(T - Torque);
 
Regressor_Matrix_Pos = Regressor_Matrix;
Regressor_Matrix_Pos(1,41:45) = 0;
Regressor_Matrix_Pos(2,46:50) = 0;
Regressor_Matrix_Pos(3,51:55) = 0;
Regressor_Matrix_Pos(4,56:60) = 0;
Regressor_Matrix_Pos(5,61:65) = 0;
Regressor_Matrix_Pos(6,66:70) = 0;
 
Regressor_Matrix_Neg = Regressor_Matrix;
Regressor_Matrix_Neg(1,11:15) = 0;
Regressor_Matrix_Neg(2,16:20) = 0;
Regressor_Matrix_Neg(3,21:25) = 0;
Regressor_Matrix_Neg(4,26:30) = 0;
Regressor_Matrix_Neg(5,31:35) = 0;
Regressor_Matrix_Neg(6,36:40) = 0;
 
end




















