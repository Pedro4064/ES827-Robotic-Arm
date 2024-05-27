function [pp_x,pp_y, pp_z] = move_L(x_t,y_t, z_t, vx, vy, vz, time, num_waypoints)
%MOVE_L Calculate the cubic piece-wise polynomials for a linear movement between ending and starting points, and 
%specified starting and ending velocities, the total number of waypoints to subedvide the overall trajectory (as
%to ensure a linear workspace trajectory), and the total time it takes for the movement to complete. The coeficientes 
%are for the following polynomial: q_i(t) = a0 + a1(t - ti) + a2(t-ti)^2 + a3(t - ti)^3, where ti is the starting
%time of that specific segment. 
%   [pp_x, pp_y, pp_z] = MOVE_L([0 10], [0 0], [0 15], [0 0], [0 0], [0 0], [0 15], (15-0)*1000) Calculate the cubic 
%piece-wise polynomial for the linear movement between the points {x=0, y=0, z=0}cm and {x=10, y=0, z=15}cm in 15 seconds, 
%with 15000 intermediate waypoints, so the movement between each waypoint takes 1ms.
%
% See also MOVE_L, PPVAL

% Generate waypoints on the line
x = linspace(x_t(1), x_t(2), num_waypoints);
y = linspace(y_t(1), y_t(2), num_waypoints);
z = linspace(z_t(1), z_t(2), num_waypoints);

% Assume equally spaced times, this makes the implementation easier, e.g you can 
times = linspace(time(1), time(2), num_waypoints);

% Boundary conditions 
initial_velocity_x = vx(1);
final_velocity_x   = vx(2);

initial_velocity_y = vy(1);
final_velocity_y   = vy(2);

initial_velocity_z = vz(1);
final_velocity_z   = vz(2);

% Compute the cubic spline with specified boundary conditions for x
pp_x = csape(times, x, 'clamped', [initial_velocity_x, final_velocity_x]);

% Compute the cubic spline with specified boundary conditions for y
pp_y = csape(times, y, 'clamped', [initial_velocity_y, final_velocity_y]);

% Compute the cubic spline with specified boundary conditions for z
pp_z = csape(times, z, 'clamped', [initial_velocity_z, final_velocity_z]);

% Generate a finer set of points for smooth plotting
t_fine = linspace(min(times), max(times), 100);
x_fine = ppval(pp_x, t_fine);
y_fine = ppval(pp_y, t_fine);
z_fine = ppval(pp_z, t_fine);

% Compute the derivatives (velocities)
pp_dx = fnder(pp_x, 1);
pp_dy = fnder(pp_y, 1);
pp_dz = fnder(pp_z, 1);

vx_fine = ppval(pp_dx, t_fine);
vy_fine = ppval(pp_dy, t_fine);
vz_fine = ppval(pp_dz, t_fine);

% Plot the 3D path
figure;
subplot(4, 1, 1);
plot3(x, y, z, 'o', x_fine, y_fine, z_fine, '-');
title('3D Cubic Spline Interpolation on a Linear Pattern');
xlabel('X');
ylabel('Y');
zlabel('Z');
legend('Waypoints', 'Cubic Spline');
grid on;
axis equal;

% Plot the velocity components vx, vy, vz
subplot(4, 1, 2);
plot(t_fine, vx_fine, '-');
title('Velocity in X Direction');
xlabel('Time');
ylabel('Velocity in X');
grid on;

subplot(4, 1, 3);
plot(t_fine, vy_fine, '-');
title('Velocity in Y Direction');
xlabel('Time');
ylabel('Velocity in Y');
grid on;

subplot(4, 1, 4);
plot(t_fine, vz_fine, '-');
title('Velocity in Z Direction');
xlabel('Time');
ylabel('Velocity in Z');
grid on;
end

