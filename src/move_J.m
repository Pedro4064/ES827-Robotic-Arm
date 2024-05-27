function [x_coe,y_coe, z_coe] = move_J(x_t,y_t, z_t, vx, vy, vz, time)
%MOVE_J Calculate the cubic polynomial for the free trajectory between starting and ending points, 
%and specified starting and ending velocity conditions.
%   [x_coe, y_coe, z_coe] = MOVE_J([0 10], [0 0], [0 15], [0 0], [0 0], [0 0], [0 15]) Calculate the polynomial
%coeficientes for the free movement between the points {x=0, y=0, z=0}cm and {x=10, y=0, z=15}cm in 15 seconds.
%the coeficientes are in the following order [a0, a1, a2, a3] such that, for example, x(t) = a0 + a1*t + a2*t.^2 + a3*t.^3
%
%   It is important to note that this returns the coeficientes in the reverse order that is used as input in the command PPVAL,
%so make sure to use FLIP
%
% See also MOVE_L, PPVAL, FLIP
t_matrix = [1 time(1)   time(1)^2   time(1)^3
            0   1     2*time(1)   3*time(1)^2
            1 time(2)   time(2)^2   time(2)^3
            0   1     2*time(2)   3*time(2)^2];

conditions_x = [x_t(1) vx(1) x_t(2) vx(2)]';
conditions_y = [y_t(1) vy(1) y_t(2) vy(2)]';
conditions_z = [z_t(1) vz(1) z_t(2) vz(2)]';

x_coe = t_matrix \ conditions_x;
y_coe = t_matrix \ conditions_y;
z_coe = t_matrix \ conditions_z;
end

