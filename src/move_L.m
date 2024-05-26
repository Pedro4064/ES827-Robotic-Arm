function [pp_x,pp_y] = move_L(x_t,y_t, time, num_waypoints)
%MOVE_L Summary of this function goes here
%   Detailed explanation goes here

% Generate waypoints on the line
x = linspace(x_t(1), x_t(2), num_waypoints);
y = linspace(y_t(1), y_t(2), num_waypoints);

% Assume equally spaced times
times = linspace(0, time, num_waypoints);

% Boundary conditions for x and y (zero velocity at start and end)
initial_velocity_x = 0;
final_velocity_x   = 0;
initial_velocity_y = 0;
final_velocity_y   = 0;

% Compute the cubic spline with specified boundary conditions for x
pp_x = csape(times, x, 'clamped', [initial_velocity_x, final_velocity_x]);

% Compute the cubic spline with specified boundary conditions for y
pp_y = csape(times, y, 'clamped', [initial_velocity_y, final_velocity_y]);

% Generate a finer set of points for smooth plotting
t_fine = linspace(min(times), max(times), 100);
x_fine = ppval(pp_x, t_fine);
y_fine = ppval(pp_y, t_fine);

% Compute the derivatives (velocities)
pp_dx = fnder(pp_x, 1);
pp_dy = fnder(pp_y, 1);
vx_fine = ppval(pp_dx, t_fine);
vy_fine = ppval(pp_dy, t_fine);

% Compute the magnitude of the velocity
v_fine = sqrt(vx_fine.^2 + vy_fine.^2);

% Plot the 2D path
figure;
subplot(3, 1, 1);
plot(x, y, 'o', x_fine, y_fine, '-');
title('2D Cubic Spline Interpolation on a Linear Pattern');
xlabel('X');
ylabel('Y');
legend('Waypoints', 'Cubic Spline');
grid on;
axis equal;

% Plot the velocity components vx and vy
subplot(3, 1, 2);
plot(t_fine, vx_fine, '-');
title('Velocity in X Direction');
xlabel('Time');
ylabel('Velocity in X');
grid on;

subplot(3, 1, 3);
plot(t_fine, vy_fine, '-');
title('Velocity in Y Direction');
xlabel('Time');
ylabel('Velocity in Y');
grid on;
end

