%% Forward Kinematics
syms l1 l2 l3 l4;                               % Symbolic link lenghts
syms theta_1 theta_2 theta_3 theta_4 theta_5;   % Symbolic joint angles

digits(4)
sympref('FloatingPointOutput',true);

A1 = link_transformation_matrix_gen(0, deg2rad(90), l1, theta_1);
A2 = link_transformation_matrix_gen(l2, 0, 0, theta_2);
A3 = link_transformation_matrix_gen(l3, 0, 0, theta_3);

A4 = link_transformation_matrix_gen(l4, 0, 0, theta_4);
A5 = link_transformation_matrix_gen(0,  0, 0, theta_5);

A_transformation_full = eval(simplify(A1*A2*A3*A4*A5));

%% Inverse Kinematics
target_eqs  = reshape(A_transformation_full(1:3, 1:4), [], 1);
inverse_jac = jacobian(target_eqs, [theta_1, theta_2, theta_3, theta_4, theta_5]);

starting_positions = [0, 0, 0, 0, 0];
%% Example of a jcaboian - Aula 12 Pag 34
test = [l1*cos(theta_1)+l2*cos(theta_1+theta_2);l1*sin(theta_1)+l2*sin(theta_1+theta_2)]
jacobian(test, [theta_1, theta_2, theta_3])

%% Define the coordinates of the starting and ending points of the first vector
quiver3(0, 0, 0, 1, 1, 1);