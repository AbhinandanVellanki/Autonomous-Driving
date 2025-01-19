% Parameters
Vx = 30; % m/s
r_positive = 1000; % m
r_negative = 500; % m
time_step = 0.01; % s

% Time vectors for each segment
t1 = 0:time_step:1; % Straight path (1 second)
t2 = 0:time_step:5; % Positive-curvature arc (5 seconds)
t3 = 0:time_step:1; % Straight path (1 second)
t4 = 0:time_step:5; % Negative-curvature arc (5 seconds)

% Yaw rates
yaw_rate_straight = 0;
yaw_rate_positive = Vx / r_positive;
yaw_rate_negative = -Vx / r_negative;

% Initialize positions and angles
X = 0;
Y = 0;
theta = 0;

% Trajectory calculation
for t = t1
    X(end+1) = X(end) + Vx * cos(theta) * time_step;
    Y(end+1) = Y(end) + Vx * sin(theta) * time_step;
end

for t = t2
    theta = theta + yaw_rate_positive * time_step;
    X(end+1) = X(end) + Vx * cos(theta) * time_step;
    Y(end+1) = Y(end) + Vx * sin(theta) * time_step;
end

for t = t3
    X(end+1) = X(end) + Vx * cos(theta) * time_step;
    Y(end+1) = Y(end) + Vx * sin(theta) * time_step;
end

for t = t4
    theta = theta + yaw_rate_negative * time_step;
    X(end+1) = X(end) + Vx * cos(theta) * time_step;
    Y(end+1) = Y(end) + Vx * sin(theta) * time_step;
end

% Plotting the trajectory
figure;
plot(X, Y, 'b', 'LineWidth', 1.5);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Vehicle Trajectory');
grid on;
legend('Trajectory');
axis equal;