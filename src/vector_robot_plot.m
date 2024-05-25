function vector_robot_plot(A1, A2, A3, A4, A5, sym_thetas, num_thetas, lengths)
%VECTOR_ROBOT_PLOT Summary of this function goes here
%   Detailed explanation goes here

% First it is necessary to substitute all variables for their numerical
% counterparts
A1_sub = subs(A1, sym_thetas(1), num_thetas(1));
A2_sub = subs(A2, sym_thetas(2), num_thetas(2));
A3_sub = subs(A3, sym_thetas(3), num_thetas(3));
A4_sub = subs(A4, sym_thetas(4), num_thetas(4));
A5_sub = subs(A5, sym_thetas(5), num_thetas(5));

% Plot first upright link (as it is alwas only Z and does not change
% representation since it only rotates around z axis)
quiver3(0, 0, 0, 0, 0, lengths(1));
hold on;

% Now calculate the vector representing the second link, by aplying the
% successive rotations of the A1 and A2 matrixes.
% By analyzing the axis assigned (when doing the DH analysis) we can see
% that on the starting position of the second link it only has componentes
% on its x axis, of lenght l2
rot_trans = A1_sub(1:3, 1:3)*A2_sub(1:3, 1:3);
l2_vector = rot_trans * [lengths(2); 0; 0];
quiver3(0, 0, lengths(1), l2_vector(1), l2_vector(2), l2_vector(3));

% Now calculate the vector representation of the third link. Since the
% first two transformation matrixes were already multiplied, re-use the
% resulting matrix and just post-multiply the third rotation matrix (from
% the A3 matrix). 
% Like its predecessor, the starting position is only on its x-axis
rot_trans = rot_trans * A3_sub(1:3, 1:3);
l3_vector = rot_trans * [lengths(3); 0; 0];
quiver3(l2_vector(1), l2_vector(2), lengths(1) + l2_vector(3), l3_vector(1), l3_vector(2), l3_vector(3));

% Calculate the fourth rotation. Since it is at the wrist of our robot it
% does not have a lenght and therefore will not be ploted, bur rather it
% will be ploted on the next step
rot_trans = rot_trans * A4_sub(1:3, 1:3);
l4_starting_points = [0; 0; lengths(1)] + l2_vector + l3_vector;
l4_vector = rot_trans * [lengths(4)+lengths(5); 0; 0];
quiver3(l4_starting_points(1), l4_starting_points(2), l4_starting_points(3), l4_vector(1), l4_vector(2), l4_vector(3));

% Calculate and plot the axis representing the end-factor of our robot

% rot_trans =  rot_trans * A5_sub(1:3, 1:3)
% end_factor_starting_points = l4_starting_points + l4_vector;
% end_factor_x_unit_vector = rot_trans * [1;0;0];
% end_factor_y_unit_vector = rot_trans * [0;1;0];
% end_factor_z_unit_vector = rot_trans * [1;0;0]

% quiver3(end_factor_starting_points(1), ...
%         end_factor_starting_points(2), ...
%         end_factor_starting_points(3), ...
%         end_factor_x_unit_vector(1), ...
%         end_factor_x_unit_vector(2), ...
%         end_factor_x_unit_vector(3));
% 
% 
% quiver3(end_factor_starting_points(1), ...
%         end_factor_starting_points(2), ...
%         end_factor_starting_points(3), ...
%         end_factor_y_unit_vector(1), ...
%         end_factor_y_unit_vector(2), ...
%         end_factor_y_unit_vector(3));

% quiver3(end_factor_starting_points(1), ...
%         end_factor_starting_points(2), ...
%         end_factor_starting_points(3), ...
%         end_factor_z_unit_vector(1), ...
%         end_factor_z_unit_vector(2), ...
%         end_factor_z_unit_vector(3));

hold off;
end

