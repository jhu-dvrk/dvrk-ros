function [joint_trajectory,j_ranges] = generate_joint_grid(joint_range)
%  Institute: The Chinese University of Hong Kong
%  Author(s):  Hongbin LIN, Vincent Hui, Samuel Au
%  Created on: 2018-10-05

    j_ranges = joint_range;
    [j7,j6,j5,j4,j3,j2,j1] = ndgrid(j_ranges{7},j_ranges{6},j_ranges{5},j_ranges{4},j_ranges{3},j_ranges{2},j_ranges{1});
    joint_trajectory = [j1(:),j2(:),j3(:),j4(:),j5(:),j6(:),j7(:)]';
end