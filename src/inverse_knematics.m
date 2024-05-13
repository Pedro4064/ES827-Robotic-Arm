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
target_eq = [diag(A_transformation_full(1:3, 1:3)); A_transformation_full(1:3, 4)]
%jacobian(A_transformation_full, [theta_1, theta_2, theta_3, theta_4, theta_5])
%% Example of a jcaboian - Aula 12 Pag 34
test = [l1*cos(theta_1)+l2*cos(theta_1+theta_2);l1*sin(theta_1)+l2*sin(theta_1+theta_2)]
jacobian(test, [theta_1, theta_2, theta_3])

%% 
syms theta_x theta_y theta_z;

rx = [1 0 0; 0 cos(theta_x) -1*sin(theta_x); 0 sin(theta_x) cos(theta_x)]
ry = [cos(theta_y) 0 sin(theta_y); 0 1 0; -1*sin(theta_y) 0 cos(theta_y)]
rz = [cos(theta_z) -1*sin(theta_z) 0; sin(theta_z) cos(theta_z) 0; 0 0 1]