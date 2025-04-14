clc; clear; close all;

% --- 🔹 Simulation Parameters ---
dt = 0.1;
N = 20;
M = 10;
L = 2.5;
T_sim = 800;

% --- 🔹 Leader Initialization ---
x_leader = [0; 0; 0; 0];
xref_leader = [30; 30; 0; 0];

% --- 🔹 Follower Initialization ---
num_followers = 3;
formation_offsets = [0, -20; -20, 0; -20, -20];
x_followers = repmat(x_leader, 1, num_followers);
v_followers = zeros(2, num_followers); % velocity for PID (x, y)

% --- 🔹 Linearized MPC Models ---
v_nominal = 10;
delta_nominal = 0;
theta_values = linspace(-pi, pi, 120);
A_models = cell(length(theta_values), 1);
B_models = cell(length(theta_values), 1);
mpc_controllers = cell(length(theta_values), 1);

for j = 1:length(theta_values)
    theta0 = theta_values(j);
    A_models{j} = [1 0 -v_nominal*sin(theta0)*dt cos(theta0)*dt;
                   0 1  v_nominal*cos(theta0)*dt sin(theta0)*dt;
                   0 0  1 (1/L)*tan(delta_nominal)*dt;
                   0 0  0 1];
    B_models{j} = [0 0;
                   0 0;
                   (v_nominal/L) * sec(delta_nominal)^2 * dt 0;
                   0 dt];
    sys = ss(A_models{j}, B_models{j}, eye(4), zeros(4,2), dt);
    mpc_controllers{j} = mpc(sys);
    mpc_controllers{j}.PredictionHorizon = N;
    mpc_controllers{j}.ControlHorizon = M;
    mpc_controllers{j}.Weights.ManipulatedVariables = [0.01 0.01];
    mpc_controllers{j}.Weights.ManipulatedVariablesRate = [0.005 0.005];
    mpc_controllers{j}.Weights.OutputVariables = [0.5 0.5 0 0.1];
    mpc_controllers{j}.MV(1).Min = -deg2rad(30);
    mpc_controllers{j}.MV(1).Max = deg2rad(30);
    mpc_controllers{j}.MV(2).Min = -4;
    mpc_controllers{j}.MV(2).Max = 4;
end

% --- 🔹 Logs ---
x_history_leader = x_leader;
x_history_followers = zeros(4, T_sim, num_followers);
u_history_leader = [];

% --- 🔹 MPC State ---
mpc_state_leader = [];

% --- 🔹 Waypoints ---
waypoints = [30, 30; 50, 10; 20, 20; 0, 0; 30, 30; 50, 10; 20, 20; 0, 0];
num_waypoints = size(waypoints, 1);
waypoint_index = 1;

% --- 🔹 PID Gains (same for all followers) ---
Kp = 2.5;
Kd = 1.2;

% --- 🔹 Simulation Loop ---
for k = 1:T_sim
    % Update leader's target every 100 steps
    if mod(k, 100) == 0 && waypoint_index < num_waypoints
        waypoint_index = waypoint_index + 1;
    end
    xref_leader = [waypoints(waypoint_index, :)'; 0; 0];

    % --- Leader MPC Control ---
    [~, theta_idx] = min(abs(theta_values - x_leader(3)));
    mpc_current = mpc_controllers{theta_idx};
    if isempty(mpc_state_leader)
        mpc_state_leader = mpcstate(mpc_current);
    end
    u_leader = mpcmove(mpc_current, mpc_state_leader, x_leader, xref_leader);
    x_leader = A_models{theta_idx} * x_leader + B_models{theta_idx} * u_leader;
    x_history_leader = [x_history_leader, x_leader];
    u_history_leader = [u_history_leader; u_leader'];

    % --- Follower PID Control ---
    for i = 1:num_followers
        % Get dynamic target (leader position + rotated offset)
        R = [cos(x_leader(3)), -sin(x_leader(3)); sin(x_leader(3)), cos(x_leader(3))];
        target_pos = x_leader(1:2) + R * formation_offsets(i,:)';

        % Compute position error
        pos_error = target_pos - x_followers(1:2, i);
        vel_error = -v_followers(:, i);

        % PID control law (acceleration command)
        acc_cmd = Kp * pos_error + Kd * vel_error;

        % Euler integration for simple velocity/position update
        v_followers(:, i) = v_followers(:, i) + acc_cmd * dt;
        x_followers(1:2, i) = x_followers(1:2, i) + v_followers(:, i) * dt;

        % Set yaw to face velocity direction (optional)
        x_followers(3, i) = atan2(v_followers(2, i), v_followers(1, i));
        x_followers(4, i) = norm(v_followers(:, i));

        x_history_followers(:, k, i) = x_followers(:, i);
    end
end

% --- 🔹 Animation ---
figure;
hold on; grid on; axis equal;
xlim([-50 60]); ylim([-30 40]);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Holonomic Follower Formation Control (MPC + PID)');

h_leader = plot(NaN, NaN, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
h_followers = gobjects(1, num_followers);
for i = 1:num_followers
    h_followers(i) = plot(NaN, NaN, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
end

for k = 1:T_sim
    set(h_leader, 'XData', x_history_leader(1,k), 'YData', x_history_leader(2,k));
    for i = 1:num_followers
        set(h_followers(i), 'XData', x_history_followers(1,k,i), 'YData', x_history_followers(2,k,i));
    end
    drawnow;
    pause(0.05);
end

disp('Animation Complete!');
