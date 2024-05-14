function vector_robot_plot(A1, A2, sym_thetas, num_thetas, lengths)
%VECTOR_ROBOT_PLOT Summary of this function goes here
%   Detailed explanation goes here

% First it is necessary to substitute all variables for their numerical
% counterparts
A1_sub = subs(A1, sym_thetas(1), num_thetas(1));
A2_sub = subs(A2, sym_thetas(2), num_thetas(2));
%A3_sub = subs(A3, theta_3, thetas(3));
%A4_sub = subs(A4, theta_4, thetas(4));
%A5_sub = subs(A5, theta_5, thetas(5));

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

hold off;
end

