% Define vehicle parameters
C_alpha_f = 80000; % Front cornering stiffness (N/rad)
C_alpha_r = 80000; % Rear cornering stiffness (N/rad)
m = 1573;          % Mass (kg)
I_z = 2873;        % Yaw moment of inertia (kg*m^2)
V_x = 60;          % Longitudinal velocity (m/s)
l_f = 1.1;         % Distance from CG to front axle (m)
l_r = 1.58;        % Distance from CG to rear axle (m)

% Define state-space matrices
A = [0, 1, 0, 0;
    -(2*C_alpha_f + 2*C_alpha_r)/(m*V_x), 0, (2*C_alpha_f + 2*C_alpha_r)/m, (-2*C_alpha_f*l_f + 2*C_alpha_r*l_r)/(m*V_x);
    0, 0, 0, 1;
    0, -(2*C_alpha_f*l_f - 2*C_alpha_r*l_r)/(I_z*V_x), (2*C_alpha_f*l_f - 2*C_alpha_r*l_r)/I_z, -(2*C_alpha_f*l_f^2 + 2*C_alpha_r*l_r^2)/(I_z*V_x)];

B1 = [0; 2*C_alpha_f/m; 0; 2*C_alpha_f*l_f/I_z];
B2 = [0; -(2*C_alpha_f*l_f - 2*C_alpha_r*l_r)/(m*V_x) - V_x; 0; -(2*C_alpha_f*l_f^2 + 2*C_alpha_r*l_r^2)/(I_z*V_x)];

% Design state feedback controller using pole placement
poles = [-9.25, -9.5, -30, -35]; % desired closed loop eigenvalues
K = place(A, B1, poles);

% Closed Loop A matrix
Acl = A - B1*K;

% Simulation Parameters
dt = 0.01; % s
t1 = 0:dt:1; % Straight path (1 second)
t2 = 0:dt:5; % Positive-curvature arc (5 seconds)
t3 = 0:dt:1; % Straight path (1 second)
t4 = 0:dt:5; % Negative-curvature arc (5 seconds)
time = 0:dt:12; % total time
X_init = 0;
Y_init = 0;

% Yaw rates
yaw_rate_straight = 0;
yaw_rate_positive = V_x / 1000; % Positive curvature (radius 1000 m)
yaw_rate_negative = -V_x / 500; % Negative curvature (radius 500 m)

% Populate Yaw rate array
psi_dot_des = [0, [yaw_rate_straight*ones(1, 1/dt)],...
    [yaw_rate_positive*ones(1, 5/dt)],...
    [yaw_rate_straight*ones(1, 1/dt)],...
    [yaw_rate_negative*ones(1, 5/dt)]];

% Calculate Yaw Values
yaw_values = zeros(1, length(psi_dot_des));

% Populate yaw values
for i = 2:length(yaw_values)
    yaw_values(i) = yaw_values(i-1)+psi_dot_des(i)*dt;
end

% Global Desired Path
X_des = zeros(1, length(psi_dot_des));
Y_des = zeros(1, length(psi_dot_des));
Y_des(1) = Y_init;
for i = 2:length(psi_dot_des)
    X_des(i) = X_des(i-1) + V_x*cos(yaw_values(i))*dt;
    Y_des(i) = Y_des(i-1) + V_x*sin(yaw_values(i))*dt;
end

% state matrix
x = zeros(4, length(psi_dot_des)); % [lateral error, lateral error rate, yaw error, yaw error rate]

% to store steering angles
delta = zeros(1, length(psi_dot_des));

% to store steering angle rates
delta_rates = zeros(1, length(psi_dot_des));

% simulate system and populate states
for i = 2:length(psi_dot_des)
    delta(i) = -K*x(:,i-1);
    delta_rates(i) = (delta(i) - delta(i-1))/dt;
    dx = Acl*x(:,i-1) + B2*psi_dot_des(i); % current step, where B2 is the scaling matrix and psi_dot_des is the desired output
    x(:,i) = x(:,i-1) + dx*dt; % computing the current state from the previous state
end

% Calculate true Y positions
Y_true = zeros(size(psi_dot_des));

for i = 1:length(psi_dot_des)
    % Convert lateral error to global frame using yaw angle error
    Y_true(i) = Y_des(i) + x(1,i) * sin(x(3,i));
end

% Calculate true X positions
X_true = zeros(size(psi_dot_des));

for i = 1:length(psi_dot_des)
    % Convert lateral error to global frame using yaw angle error
    X_true(i) = X_des(i) + x(1,i) * cos(x(3,i));
end

% Plot results
figure;
plot(time, x(1,:), 'b', 'DisplayName', 'Lateral Position Error (e1) m');
hold on;
xlabel('Time (s)');
ylabel('Lateral Error');
title('System Response');
legend;
grid on;

figure;
hold on;
plot(time, x(3,:), 'r', 'DisplayName', 'Yaw Angle Error (e2) rad');
xlabel('Time (s)');
ylabel('Yaw Angle Error');
title('System Response');
legend;
grid on;

figure;
plot(time, delta_rates, 'g', 'DisplayName', 'Steering Input Rate (δ/dt)');
xlabel('Time (s)');
ylabel('Steering Input Rate(rad/s)');
title('Steering Input Rate vs Time');
legend;
grid on;

figure;
plot(X_true, Y_true, 'b', 'DisplayName', 'True Path');
hold on;
plot(X_des, Y_des, 'r--', 'DisplayName', 'Desired Path');
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Vehicle Path in XY Plane');
legend;
grid on;

% Zoom in on transition points
figure;
plot(X_true(X_true < 40), Y_true(X_true < 40), 'b', X_des(X_des < 40), Y_des(X_des < 40), 'r--');
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Zoomed Path: Initial Transition');
legend('True Path','Desired Path')
grid on;

figure;
plot(X_true(X_true > 200), Y_true(X_true > 200), 'b', X_des(X_des > 200), Y_des(X_des > 200), 'r--');
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Zoomed Path: Final Transition');
legend('True Path','Desired Path')
grid on;
