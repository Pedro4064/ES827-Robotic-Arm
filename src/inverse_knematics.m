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

%% Trajectory Planning - Circular Motion

% Trapezoidal Velocity Profile
total_time = 30;                            % In seconds
acceleration_time   = (total_time / 2)/2;   % Ramp up
deceleration_time   = acceleration_time;    % Ramp Down
const_velocity_time = total_time / 2;       % Constant velocity

radius = 0.25;               % In meters
max_velocity = deg2rad(30); % Max angular velocity of 30deg/s in rad/s

points = 1000;
time = linspace(0, total_time, points);
angular_velocities = zeros(1, points);

angular_velocities(time <=acceleration_time) = (max_velocity/acceleration_time) * time(time <= acceleration_time);
angular_velocities(time > acceleration_time & time <= const_velocity_time + acceleration_time) = max_velocity;
angular_velocities(time > const_velocity_time + acceleration_time) = max_velocity - ((max_velocity / deceleration_time) * (time(time>acceleration_time+const_velocity_time) - (acceleration_time + const_velocity_time)));

plot(time, angular_velocities);

%% Now we can decompose the angular velocity into X and Y velocities
angles = cumtrapz(time, angular_velocities);
x_velocity = radius * angular_velocities .* sin(angles);
y_velocity = radius * angular_velocities .* cos(angles);

plot(time, x_velocity, time, y_velocity);
legend('x Velocity', 'y Velocity');
%%
x_integral = cumtrapz(time, x_velocity);
y_integral = cumtrapz(time, y_velocity);

scatter(x_integral, y_integral);
axis equal;

%%
% Define the waypoints
waypoints = [0, 1, 4, 9, 16]; % example waypoints

% Assume equally spaced times
num_points = length(waypoints);
times = linspace(0, num_points-1, num_points);

% Boundary conditions
initial_velocity = 0;
initial_acceleration = 0;
final_velocity = 0;
final_acceleration = 0;

% Compute the cubic spline with specified boundary conditions
% Using the csape function to specify the clamped boundary conditions
pp = csape(times, waypoints, 'clamped', [initial_velocity, final_velocity]);

% Generate a finer set of points for smooth plotting
t_fine = linspace(min(times), max(times), 100);
y_fine = ppval(pp, t_fine);

% Plot the results
figure;
plot(times, waypoints, 'o', t_fine, y_fine, '-');
title('Cubic Spline Interpolation with Clamped Boundary Conditions');
xlabel('Time');
ylabel('Position');
legend('Waypoints', 'Cubic Spline');
grid on;
%%
% Define the 2D waypoints
waypoints = [0, 0; 1, 2; 4, 5; 9, 7; 16, 10]; % example 2D waypoints

% Separate the waypoints into x and y components
x = waypoints(:, 1);
y = waypoints(:, 2);

% Assume equally spaced times
num_points = length(x);
times = linspace(0, num_points-1, num_points);

% Boundary conditions
initial_velocity_x = 0;
final_velocity_x = 0;
initial_velocity_y = 0;
final_velocity_y = 0;

% Compute the cubic spline with specified boundary conditions for x
pp_x = csape(times, x, 'clamped', [initial_velocity_x, final_velocity_x]);

% Compute the cubic spline with specified boundary conditions for y
pp_y = csape(times, y, 'clamped', [initial_velocity_y, final_velocity_y]);

% Generate a finer set of points for smooth plotting
t_fine = linspace(min(times), max(times), 100);
x_fine = ppval(pp_x, t_fine);
y_fine = ppval(pp_y, t_fine);

% Plot the results
figure;
plot(x, y, 'o', x_fine, y_fine, '-');
title('2D Cubic Spline Interpolation with Clamped Boundary Conditions');
xlabel('X');
ylabel('Y');
legend('Waypoints', 'Cubic Spline');
grid on;
axis equal;
%%
% Parameters for the circular pattern
radius = 5;
num_waypoints = 100; % number of waypoints on the circle
theta = linspace(0, 2*pi, num_waypoints + 1); % angles for the waypoints
theta(end) = []; % remove the last point to avoid duplication

% Generate waypoints on the circle
x = radius * cos(theta);
y = radius * sin(theta);

% Assume equally spaced times
times = linspace(0, num_waypoints-1, num_waypoints);

% Boundary conditions for x and y (zero velocity at start and end)
initial_velocity_x = 0;
final_velocity_x = 0;
initial_velocity_y = 0;
final_velocity_y = 0;

% Compute the cubic spline with specified boundary conditions for x
pp_x = csape(times, x, 'clamped', [initial_velocity_x, final_velocity_x]);

% Compute the cubic spline with specified boundary conditions for y
pp_y = csape(times, y, 'clamped', [initial_velocity_y, final_velocity_y]);

% Generate a finer set of points for smooth plotting
t_fine = linspace(min(times), max(times), 100);
x_fine = ppval(pp_x, t_fine);
y_fine = ppval(pp_y, t_fine);

% Plot the results
figure;
plot(x, y, 'o', x_fine, y_fine, '-');
title('2D Cubic Spline Interpolation on a Circular Pattern');
xlabel('X');
ylabel('Y');
legend('Waypoints', 'Cubic Spline');
grid on;
axis equal;
%%

% Parameters for the circular pattern
radius = 5;
num_waypoints = 100; % number of waypoints on the circle
theta = linspace(0, 2*pi, num_waypoints + 1); % angles for the waypoints
theta(end) = []; % remove the last point to avoid duplication

% Generate waypoints on the circle
x = radius * cos(theta);
y = radius * sin(theta);

% Assume equally spaced times
times = linspace(0, num_waypoints-1, num_waypoints);

% Boundary conditions for x and y (zero velocity at start and end)
initial_velocity_x = 0;
final_velocity_x = 0;
initial_velocity_y = 0;
final_velocity_y = 0;

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
subplot(2, 1, 1);
plot(x, y, 'o', x_fine, y_fine, '-');
title('2D Cubic Spline Interpolation on a Circular Pattern');
xlabel('X');
ylabel('Y');
legend('Waypoints', 'Cubic Spline');
grid on;
axis equal;

% Plot the velocity magnitude
subplot(2, 1, 2);
plot(t_fine, v_fine, '-');
title('Velocity Magnitude along the Path');
xlabel('Time');
ylabel('Velocity Magnitude');
grid on;

%%
% Parameters for the circular pattern
radius = 5;
num_waypoints = 50; % number of waypoints on the circle
theta = linspace(0, 2*pi, num_waypoints + 1); % angles for the waypoints
theta(end) = []; % remove the last point to avoid duplication

% Generate waypoints on the circle
x = radius * cos(theta);
y = radius * sin(theta);

% Assume equally spaced times
num_points = length(x);
times = linspace(0, num_waypoints-1, num_waypoints);

% Boundary conditions for x and y (zero velocity at start and end)
initial_velocity_x = 0;
final_velocity_x = 0;
initial_velocity_y = 0;
final_velocity_y = 0;

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
title('2D Cubic Spline Interpolation on a Circular Pattern');
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
