clc; clear; close all;

% --- 🔹 Simulation Parameters ---
dt = 0.1;       % Time step (s)
N = 20;         % Prediction Horizon
M = 10;         % Control Horizon
L = 2.5;        % Wheelbase (m)
T_sim = 150;    % Simulation time steps

% --- 🔹 Define Leader's Initial Conditions ---
x_leader = [0; 0; 0; 0];  % Initial position (x, y, theta, velocity)
xref_leader = [30; 30; 0; 0];  % Target state for the leader

% --- 🔹 Define Follower MAVs (Offsets) ---
num_followers = 3; % Number of followers
formation_offsets = [ % (x, y) offsets relative to leader
    0, -20;
    -20, 0;
    -20, -20]; 

% Initialize follower states
x_followers = repmat(x_leader, 1, num_followers); % Followers start at the leader's position

% --- 🔹 Define Operating Points for Linearization ---
v_nominal = 5;  
delta_nominal = 0;  
theta_values = linspace(-pi, pi, 120);  

% --- 🔹 Initialize Precomputed Linearized Models ---
A_models = cell(length(theta_values), 1);
B_models = cell(length(theta_values), 1);
mpc_controllers = cell(length(theta_values), 1);

% --- 🔹 Precompute Linearized Models for Different θ (Heading Angles) ---
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

    % Create an MPC controller for each θ
    sys = ss(A_models{j}, B_models{j}, eye(4), zeros(4,2), dt);
    mpc_controllers{j} = mpc(sys);
    mpc_controllers{j}.PredictionHorizon = N;
    mpc_controllers{j}.ControlHorizon = M;
    mpc_controllers{j}.Weights.ManipulatedVariables = [0.1 0.1];
    mpc_controllers{j}.Weights.ManipulatedVariablesRate = [0.01 0.01];
    mpc_controllers{j}.Weights.OutputVariables = [1 1 0 0.1];
    mpc_controllers{j}.MV(1).Min = -deg2rad(30);
    mpc_controllers{j}.MV(1).Max = deg2rad(30);
    mpc_controllers{j}.MV(2).Min = -2;
    mpc_controllers{j}.MV(2).Max = 2;
end

% --- 🔹 Initialize Logs ---
x_history_leader = x_leader;
x_history_followers = zeros(4, T_sim, num_followers); % Fixed storage issue
u_history_leader = [];
u_history_followers = cell(1, num_followers);

% --- 🔹 Initialize MPC State (Before Loop) ---
mpc_state_leader = [];
mpc_state_followers = cell(1, num_followers);

for k = 1:T_sim
    % --- Leader MPC Control ---
    [~, theta_idx] = min(abs(theta_values - x_leader(3)));  
    theta_idx = max(1, min(theta_idx, length(theta_values)));  
    mpc_current = mpc_controllers{theta_idx};

    if isempty(mpc_state_leader) || ~isa(mpc_state_leader, 'mpcstate')
        mpc_state_leader = mpcstate(mpc_current);
    end

    u_leader = mpcmove(mpc_current, mpc_state_leader, x_leader, xref_leader);
    u_leader = u_leader(:);
    x_leader = A_models{theta_idx} * x_leader + B_models{theta_idx} * u_leader;
    x_history_leader = [x_history_leader, x_leader];
    u_history_leader = [u_history_leader; u_leader'];

    % --- Followers' MPC Control ---
    for i = 1:num_followers
        % Compute dynamic target for follower
        xref_follower = x_leader(1:2) + formation_offsets(i,:)';
        xref_follower = [xref_follower; x_leader(3:4)]; % Maintain same heading and velocity

        [~, theta_idx] = min(abs(theta_values - x_followers(3, i)));
        theta_idx = max(1, min(theta_idx, length(theta_values)));
        mpc_current = mpc_controllers{theta_idx};

        if isempty(mpc_state_followers{i}) || ~isa(mpc_state_followers{i}, 'mpcstate')
            mpc_state_followers{i} = mpcstate(mpc_current);
        end

        u_follower = mpcmove(mpc_current, mpc_state_followers{i}, x_followers(:, i), xref_follower);
        u_follower = u_follower(:);
        x_followers(:, i) = A_models{theta_idx} * x_followers(:, i) + B_models{theta_idx} * u_follower;
        x_history_followers(:, k, i) = x_followers(:, i);
        u_history_followers{i} = [u_history_followers{i}; u_follower'];
    end
end

% --- 🔹 Plot Trajectories ---
figure;
plot(x_history_leader(1,:), x_history_leader(2,:), 'r--', 'LineWidth', 1.5); hold on;
for i = 1:num_followers
    plot(squeeze(x_history_followers(1,:,i)), squeeze(x_history_followers(2,:,i)), 'b--', 'LineWidth', 1.5);
end
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Leader-Follower Formation Control with MPC');
legend('Leader', 'Followers');
% Plot final target positions for leader and followers
plot(xref_leader(1), xref_leader(2), 'rx', 'MarkerSize', 12, 'LineWidth', 2); % Leader's target
for i = 1:num_followers
    xref_follower_final = xref_leader(1:2) + formation_offsets(i,:)';
    plot(xref_follower_final(1), xref_follower_final(2), 'gx', 'MarkerSize', 12, 'LineWidth', 2); % Follower targets
end
legend('Leader', 'Followers', 'Leader Target', 'Follower Targets');
grid on;
