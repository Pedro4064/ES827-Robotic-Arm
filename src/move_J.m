function [x_coe,y_coe, z_coe] = move_J(x_t,y_t, z_t, vx, vy, vz, time)
%MOVE_J Summary of this function goes here
%   Detailed explanation goes here
t_matrix = [1 time(1)   time(1)^2   time(1)^3
            0   1     2*time(1)   3*time(1)^2
            1 time(2)   time(2)^2   time(2)^3
            0   1     2*time(2)   3*time(2)^2
           ];

conditions_x = [x_t(1) vx(1) x_t(2) vx(2)]';
conditions_y = [y_t(1) vy(1) y_t(2) vy(2)]';
conditions_z = [z_t(1) vz(1) z_t(2) vz(2)]';

x_coe = t_matrix \ conditions_x;
y_coe = t_matrix \ conditions_y;
z_coe = t_matrix \ conditions_z;
end

