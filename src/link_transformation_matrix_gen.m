function a_matrix = link_transformation_matrix_gen(a, alpha, d, theta)
%LINK_TRANSFORMATION_MATRIX_GEN Summary of this function goes here
%   Detailed explanation goes here

a_matrix(1,1) = cos(theta);
a_matrix(1,2) = -1*sin(theta)*cos(alpha);
a_matrix(1,3) = sin(theta)*sin(alpha);
a_matrix(1,4) = a*cos(theta);

a_matrix(2,1) = sin(theta);
a_matrix(2,2) = cos(theta)*cos(alpha);
a_matrix(2,3) = -1*cos(theta)*sin(alpha);
a_matrix(2,4) = a*sin(theta);

a_matrix(3,2) = sin(alpha);
a_matrix(3,3) = cos(alpha);
a_matrix(3,4) = d;

a_matrix(4,4) = 1;
end

