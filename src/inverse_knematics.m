%% Forward Kinematics
syms l1 l2 l3 l4 l5;                            % Symbolic link lenghts
syms theta_1 theta_2 theta_3 theta_4 theta_5;   % Symbolic joint angles

digits(4)
sympref('FloatingPointOutput',true);

A1 = link_transformation_matrix_gen(0, deg2rad(90), l1, theta_1);
A2 = link_transformation_matrix_gen(l2, 0, 0, theta_2);
A3 = link_transformation_matrix_gen(l3, 0, 0, theta_3);

A4 = link_transformation_matrix_gen(0, deg2rad(-90), 0, theta_4);
A5 = link_transformation_matrix_gen(0,  0, l4+l5, theta_5);

A_transformation_full = eval(simplify(A1*A2*A3*A4*A5));
%% Plot Robot Representation
vector_robot_plot(A1, A2, A3, A4, A5, [theta_1 theta_2 theta_3 theta_4 theta_5], [0 0 0 0 pi/4], [5 5 5 5 5])

%% Inverse Knimatics

% Theta 1 - Inversa
a_1 = @(x, y) atan2(y,x);

% Theta 2 - Inversa
S = @(z) z - l1;
R = @(x, y) sqrt(x^2 + y^2);
A = @(r, s) sqrt(s^2 + (r - l4)^2);
B = @(d) l3 * sin(a_3(d));

a_2 = @(s, r, a, b) atan2(s, (r - l4)) - atan2((b/a), sqrt(1 - (b/a)^2));

% Theta 3 - Inversa
D = @(x, y, z) (((z - l1)^2 + (sqrt(x^2 + y^2) - l4)^2) - l2^2 - l5^2) / (2*l2*l3); 
a_3 = @(d) atan2(sqrt(1 - d^2), d);

% Theta 4 - Inversa
a_4 = @() theta_2 + theta_3;