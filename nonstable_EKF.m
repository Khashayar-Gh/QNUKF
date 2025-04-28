clear;
clc;
close all

% Define system parameters
g = 9.81;  % gravity
L = 1.0;   % length of pendulum
dt = 0.1;  % time step

% Define the state transition and measurement functions
f = @(x, u) [x(1) + x(2)*dt;
             x(2) + (g/L)*sin(x(1))*dt + u*dt];  % State transition (discretized)
h = @(x) x(1)^2 + x(2);  % Nonlinear measurement model

% Time steps
n_steps = 600;

% Initial state and covariance
x_init = [0.1; 0];   % Initial state estimate [theta, theta_dot]
P_init = eye(2);     % Initial state covariance (2x2 identity)
Q = 8 * eye(2);   % Process noise covariance
R = 8;             % Measurement noise covariance

% Control input (stabilizing control for testing)
u = 0.01 * ones(1, n_steps);  % Small constant control input

% True initial state
x_true = [0.1; 0];  % Actual initial state (inverted position)
x_true_vals = zeros(2, n_steps);
x_true_vals(:, 1) = x_true;

% Measurements and true states
z = zeros(1, n_steps);  % Measurements

% Simulate the inverted pendulum system
for k = 2:n_steps
    % Simulate true state evolution (nonlinear system)
    x_true = f(x_true_vals(:, k-1), u(k));
    x_true_vals(:, k) = x_true;
    
    % Simulate noisy measurement
    z(k) = h(x_true) + sqrt(R) * randn;
end

% Run EKF
[x_est_vals, P_est_vals] = EKF(f, h, x_init, P_init, Q*1e-8, R*1e-8, z, u);

% Plot results
% figure;
% hold on;
% plot(1:n_steps, x_true_vals(1, :), 'r', 'DisplayName', 'True state');
% plot(1:n_steps+1, x_est_vals(1, :), 'b', 'DisplayName', 'Estimated state');
% legend('Location', 'northwest');
% xlabel('Time step');
% ylabel('state');

figure;
plot(1:n_steps, x_true_vals(1, :) - x_est_vals(1, 1:end-1), 'r', 'LineWidth', 1.5);
xlabel('Time Step', 'FontSize', 12);
ylabel('Estimation Error', 'FontSize', 12);
grid on;





function [x_est, P_est] = EKF(f, h, x_init, P_init, Q, R, z, u)
    % Input:
    % f - function handle for nonlinear state transition: x_k+1 = f(x_k, u_k)
    % h - function handle for nonlinear measurement model: z_k = h(x_k)
    % x_init - initial state estimate
    % P_init - initial covariance matrix
    % Q - process noise covariance
    % R - measurement noise covariance
    % z - measurements
    % u - control input (if applicable)

    % Initialize state and covariance
    x_est_ = x_init*30; 
    x_est = x_est_;  
    P_est = P_init;  
    n = length(x_init);  % state vector size
    
    for k = 1:length(z)
        % Prediction step
        F_k = jacobian_f(f, x_est_, u(k));  % Linearization (Jacobian) of f around x_est
        x_pred = f(x_est_, u(k));           % Nonlinear state prediction
        P_pred = F_k * P_est * F_k' + Q; % Covariance prediction
        
        % Update step
        H_k = jacobian_h(h, x_pred);    % Linearization (Jacobian) of h around x_pred
        z_pred = h(x_pred);             % Measurement prediction
        y = z(k) - z_pred;              % Measurement residual
        
        S_k = H_k * P_pred * H_k' + R;  % Innovation covariance
        K_k = P_pred * H_k' / S_k;      % Kalman gain
        
        x_est_ = x_pred + K_k * y;       % State update
        P_est = (eye(n) - K_k * H_k) * P_pred;  % Covariance update
        x_est = [x_est ,x_est_];
    end
end

% Jacobian of the state transition function
function F_k = jacobian_f(f, x, u)
    eps = 1e-5;
    F_k = zeros(length(x), length(x));
    fx = f(x, u);
    for i = 1:length(x)
        x_eps = x;
        x_eps(i) = x_eps(i) + eps;
        F_k(:, i) = (f(x_eps, u) - fx) / eps;
    end
end

% Jacobian of the measurement function
function H_k = jacobian_h(h, x)
    eps = 1e-5;
    H_k = zeros(length(h(x)), length(x));
    hx = h(x);
    for i = 1:length(x)
        x_eps = x;
        x_eps(i) = x_eps(i) + eps;
        H_k(:, i) = (h(x_eps) - hx) / eps;
    end
end
