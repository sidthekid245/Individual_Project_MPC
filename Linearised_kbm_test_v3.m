clc; clear; close all;

% --- 🔹 Simulation Parameters ---
dt = 0.1;       % Time step (s)
L = 2.5;        % Wheelbase (m)
T_sim = 100;    % Number of simulation steps
N_align = 10;    % Realign every N_align steps
delta_nominal = 0; % Nominal steering angle

% --- 🔹 Define Operating Points for `v` (Velocity) and `θ` (Heading) ---
v_values = linspace(0, 20, 10);   % 10 velocity linearization points
theta_values = linspace(-pi, pi, 120);  % 120 heading linearization points

% Initialize cell arrays for storing precomputed linearized models
A_models = cell(length(v_values), length(theta_values));
B_models = cell(length(v_values), length(theta_values));

% --- 🔹 Precompute Linearized Models at Different `(v, θ)` Values ---
for i = 1:length(v_values)
    for j = 1:length(theta_values)
        v0 = v_values(i);
        theta0 = theta_values(j);
        
        % Compute A matrix using `v0` and `theta0`
        A_models{i,j} = [1 0 -v0*sin(theta0)*dt cos(theta0)*dt;
                         0 1  v0*cos(theta0)*dt sin(theta0)*dt;
                         0 0  1 (1/L)*tan(delta_nominal)*dt;
                         0 0  0 1];

        % Compute B matrix using nominal `δ = 0`
        B_models{i,j} = [0 0;
                         0 0;
                         (v0/L) * sec(delta_nominal)^2 * dt 0;
                         0 dt];
    end
end

% --- 🔹 Generate Random Steering Angle Signal (±15°) ---
delta_signal = deg2rad(-15) + (deg2rad(30) * rand(1, T_sim));  % Random values in [-15°, 15°]
a_signal = zeros(1, T_sim); % Acceleration is kept at zero

% --- 🔹 Initialize State Variables ---
v_nominal = 5;  % Start with a nominal velocity
x_linear = [0; 0; 0; v_nominal];  % State for precomputed `(v, θ)` linearized model
x_nonlinear = [0; 0; 0; v_nominal]; % State for nonlinear model

x_history_linear = x_linear;
x_history_nonlinear = x_nonlinear;
delta_history = [];
a_history = [];

% --- 🔹 Initialize Error Storage ---
error_x = [];
error_y = [];
error_theta = [];

% --- 🔹 Run Simulation ---
for k = 1:T_sim
    % --- 🔹 Find Closest Precomputed `(v, θ)` Model ---
    [~, v_idx] = min(abs(v_values - x_linear(4)));  % Closest velocity model
    [~, theta_idx] = min(abs(theta_values - x_linear(3)));  % Closest heading

    % Ensure indices are within valid bounds
    v_idx = max(1, min(v_idx, length(v_values)));
    theta_idx = max(1, min(theta_idx, length(theta_values)));

    % Select corresponding precomputed matrices
    A_k = A_models{v_idx, theta_idx};
    B_k = B_models{v_idx, theta_idx};

    % Get current random steering angle
    delta_k = delta_signal(k);
    u_k = [delta_k; a_signal(k)];  

    % --- 2️⃣ Nonlinear Kinematic Bicycle Model (Updated First) ---
    theta_nl = x_nonlinear(3);
    x_nonlinear = x_nonlinear + dt * [
        x_nonlinear(4) * cos(theta_nl);  % dx/dt = v * cos(theta)
        x_nonlinear(4) * sin(theta_nl);  % dy/dt = v * sin(theta)
        (x_nonlinear(4) / L) * tan(delta_k);  % dtheta/dt = v/L * tan(delta)
        0]; % Constant velocity

    % --- 🔹 State Realignment Every `N_align` Steps (Now After Nonlinear Update) ---
    if mod(k, N_align) == 0
        x_linear = x_nonlinear; % Reset linear model to match nonlinear model
    end

    % --- 1️⃣ Apply the Precomputed `(v, θ)` Linearized Model ---
    x_linear(3) = x_linear(3) + (x_linear(4) / L) * tan(delta_k) * dt; % Update heading correctly
    x_linear(1) = x_linear(1) + x_linear(4) * cos(x_linear(3)) * dt;   % Update X correctly
    x_linear(2) = x_linear(2) + x_linear(4) * sin(x_linear(3)) * dt;   % Update Y correctly

    % Store trajectory and control inputs
    x_history_linear = [x_history_linear, x_linear];
    x_history_nonlinear = [x_history_nonlinear, x_nonlinear];
    delta_history = [delta_history, delta_k];
    a_history = [a_history, a_signal(k)];

    % --- 🔹 Compute Error ---
    error_x = [error_x, x_nonlinear(1) - x_linear(1)];
    error_y = [error_y, x_nonlinear(2) - x_linear(2)];
    error_theta = [error_theta, x_nonlinear(3) - x_linear(3)];
end


% --- 🔹 Compute RMSE ---
RMSE_x = sqrt(mean(error_x.^2));
RMSE_y = sqrt(mean(error_y.^2));
RMSE_theta = sqrt(mean(error_theta.^2));

fprintf('RMSE_x: %.4f m\n', RMSE_x);
fprintf('RMSE_y: %.4f m\n', RMSE_y);
fprintf('RMSE_theta: %.4f rad\n', RMSE_theta);

% --- 🔹 Figure 1: Trajectory Comparison ---
figure;
plot(x_history_linear(1,:), x_history_linear(2,:), 'b--', 'LineWidth', 1.5);
hold on;
plot(x_history_nonlinear(1,:), x_history_nonlinear(2,:), 'r--', 'LineWidth', 1.5);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Comparison: Precomputed (v, θ) Linearization vs Nonlinear Model');
legend('Precomputed (v, θ) Linear Model', 'Nonlinear Model');
grid on;

% --- 🔹 Figure 2: Control Inputs ---
figure;

% Subplot 1: Steering Angle Input Over Time
subplot(3,1,1);
plot(1:T_sim, rad2deg(delta_history), 'k-', 'LineWidth', 1.5);
xlabel('Time Step');
ylabel('Steering Angle (deg)');
title('Randomized Steering Angle Input');
grid on;

% Subplot 2: Acceleration Input Over Time
subplot(3,1,2);
plot(1:T_sim, a_history, 'm-', 'LineWidth', 1.5);
xlabel('Time Step');
ylabel('Acceleration (m/s^2)');
title('Acceleration Input (Should Be Zero)');
grid on;

% --- 🔹 Figure 3: Error Analysis ---
figure;

% Subplot 1: X-Position Error
subplot(3,1,1);
plot(1:T_sim, error_x, 'b-', 'LineWidth', 1.5);
xlabel('Time Step');
ylabel('Error in X (m)');
title('Error in X Position');
grid on;

% Subplot 2: Y-Position Error
subplot(3,1,2);
plot(1:T_sim, error_y, 'r-', 'LineWidth', 1.5);
xlabel('Time Step');
ylabel('Error in Y (m)');
title('Error in Y Position');
grid on;

% Subplot 3: Theta Error
subplot(3,1,3);
plot(1:T_sim, rad2deg(error_theta), 'g-', 'LineWidth', 1.5);
xlabel('Time Step');
ylabel('Error in Heading Angle (deg)');
title('Error in Heading Angle');
grid on;
