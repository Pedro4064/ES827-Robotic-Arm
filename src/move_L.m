function [pp_x,pp_y, pp_z] = move_L(x_t,y_t, z_t, vx, vy, vz, time, num_waypoints)
%MOVE_L Summary of this function goes here
%   Detailed explanation goes here

% Generate waypoints on the line
x = linspace(x_t(1), x_t(2), num_waypoints);
y = linspace(y_t(1), y_t(2), num_waypoints);
z = linspace(z_t(1), z_t(2), num_waypoints);

% Assume equally spaced times
times = linspace(0, time(2), num_waypoints);

% Boundary conditions for x and y (zero velocity at start and end)
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

% Plot the velocity components vx and vy
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

