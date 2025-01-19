% create an object of the pepy_model class and plot its trajectory

% Set model parameters
l_f = 1.5; %[m]
l_r = 1.5; %[m]
velocity = 10.0; %[m/s]
max_steering_angle = pi/4; %[radian]
model = pepy_model(l_f, l_r, max_steering_angle, velocity);

% Trial parameters
time_step = 0.1; %[s]
end_time = 50.0; %[s]
t = 0:time_step:end_time;
step_count = length(t);

% Arrays to store results
x = zeros(1, step_count);
y = zeros(1, step_count);
yaw = zeros(1, step_count);

% Set constant steering angle
% delta = pi/4;

% Trial
for i = 1:step_count
    %delta = 10 * sin(2*pi*0.1*t(i)); % Sinusoidal Steering angle
    delta = 1 * square(2 * pi * 0.5 * t(i)); % Square Wave Steering angle

    model = model.step(time_step, delta);

    % Get and store the state
    [x(i), y(i), yaw(i)] = model.get_state();
end

% Plot the trajectory
figure;
plot(x, y, 'b-', 'LineWidth', 2)
xlabel('X [m]')
ylabel('Y [m]')
title('Vehicle Trajectory - Square Wave Steering Angle:')

% Add start and end markers
hold on
plot(x(1), y(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g')  % Start point
plot(x(step_count), y(step_count), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r')  % End point
legend('Trajectory', 'Start', 'End')

% Add arrow to show direction
quiver(x(end), y(end), cos(yaw(end)), sin(yaw(end)), 0.5, 'k-', 'LineWidth', 2, 'MaxHeadSize', 0.5)

% Display some information
text(x(1), y(1)-1, 'Start', 'HorizontalAlignment', 'center')
text(x(end), y(end)+1, 'End', 'HorizontalAlignment', 'center')

hold off