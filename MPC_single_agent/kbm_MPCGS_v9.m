clc; clear; close all;

% --- 🔹 Simulation Parameters ---
dt = 0.1;       % Time step (s)
N = 20;         % Prediction Horizon
M = 10;         % Control Horizon
L = 2.5;        % Wheelbase (m)

x = [0; 0; 0; 0];  % Initial state
xref = [10; 10; 0; 0];  % Target state
T_sim = 100;  % Simulation time steps

% --- 🔹 Define Operating Points ---
v_nominal = 10;  % Fixed operating point for velocity
delta_nominal = 0;  % Fixed operating point for steering angle
theta_values = linspace(-pi, pi, 120);  % 120 heading angle operating points

% --- 🔹 Initialize Storage for Precomputed Linearized Models ---
A_models = cell(length(theta_values), 1);
B_models = cell(length(theta_values), 1);

% --- 🔹 Precompute Linearized Models at Different θ (Heading Angles) ---
for j = 1:length(theta_values)
    theta0 = theta_values(j);
    
    % Compute linearized A matrix
    A_models{j} = [1 0 -v_nominal*sin(theta0)*dt cos(theta0)*dt;
                   0 1  v_nominal*cos(theta0)*dt sin(theta0)*dt;
                   0 0  1 (1/L)*tan(delta_nominal)*dt;
                   0 0  0 1];

    % Compute linearized B matrix
    B_models{j} = [0 0;
                   0 0;
                   (v_nominal/L) * sec(delta_nominal)^2 * dt 0;
                   0 dt];
end

mpc_controllers = cell(length(theta_values), 1);

% --- 🔹 Create an MPC Controller for Each θ (Heading Angle) ---
for j = 1:length(theta_values)
    sys = ss(A_models{j}, B_models{j}, eye(4), zeros(4,2), dt);
    mpc_controllers{j} = mpc(sys);

    % Set Prediction Horizon & Control Horizon
    mpc_controllers{j}.PredictionHorizon = N;
    mpc_controllers{j}.ControlHorizon = M;

    % Define MPC Tuning Parameters
    mpc_controllers{j}.Weights.ManipulatedVariables = [0.01 0.01]; % Penalize control effort
    mpc_controllers{j}.Weights.ManipulatedVariablesRate = [0.005 0.005]; % Penalize fast control changes
    mpc_controllers{j}.Weights.OutputVariables = [0.5 0.5 0 0.1]; % Penalize deviation from target

    % Set Constraints
    mpc_controllers{j}.MV(1).Min = -deg2rad(30);
    mpc_controllers{j}.MV(1).Max = deg2rad(30);
    mpc_controllers{j}.MV(2).Min = -4;
    mpc_controllers{j}.MV(2).Max = 4;
end

% --- 🔹 Initial Conditions ---
x_history = x;
u_history = [];
delta_history = [0];
a_history = [];

% --- 🔹 Initialize MPC State (Before Loop) ---
mpc_state = [];

for k = 1:T_sim
    % Find closest precomputed θ (heading angle) model
    [~, theta_idx] = min(abs(theta_values - x(3)));  % Closest heading model

    % Ensure index is within valid bounds
    theta_idx = max(1, min(theta_idx, length(theta_values)));

    % Select corresponding MPC controller
    mpc_current = mpc_controllers{theta_idx};

    % Update or create state tracker for this MPC controller
    if isempty(mpc_state) || ~isa(mpc_state, 'mpcstate')
        mpc_state = mpcstate(mpc_current);
    end

    % Compute optimal control inputs over the prediction horizon
    u_mpc = mpcmove(mpc_current, mpc_state, x, xref);
    
    % Ensure u_mpc is a column vector (2x1)
    u_mpc = u_mpc(:);

    % Apply only the first control input
    x = A_models{theta_idx} * x + B_models{theta_idx} * u_mpc;

    % Store trajectory and control input history
    x_history = [x_history, x];
    u_history = [u_history; u_mpc'];
    
    % Store individual control inputs
    delta_history = [delta_history, u_mpc(1)]; % Steering angle
    a_history = [a_history, u_mpc(2)]; % Acceleration
end

% --- 🔹 Figure 1: Trajectory Plot ---
figure;
plot(x_history(1,:), x_history(2,:), 'bo-', 'LineWidth', 1.5);
hold on;
plot(xref(1), xref(2), 'rx', 'MarkerSize', 10, 'LineWidth', 1.5);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('MPC Path with Gain Scheduling (Precomputed θ Models)');
grid on;
legend('Path', 'Target');

% --- 🔹 Figure 2: Control Inputs Plot ---
figure;
subplot(2,1,1);
plot(1:length(delta_history), rad2deg(delta_history), 'k-', 'LineWidth', 1.5);
xlabel('Time Step');
ylabel('Steering Angle (deg)');
title('MPC Steering Angle Input Over Time');
grid on;

subplot(2,1,2);
plot(1:length(a_history), a_history, 'm-', 'LineWidth', 1.5);
xlabel('Time Step');
ylabel('Acceleration (m/s^2)');
title('MPC Acceleration Input Over Time');
grid on;
