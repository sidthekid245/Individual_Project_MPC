clc; clear; close all;

% --- 🔹 Simulation Parameters ---
dt = 0.1;
N = 20;
M = 10;
L = 2.5;
T_sim = 1200;

% --- 🔹 Initial Conditions ---
x_leader = [0; 0; 0; 0];
num_followers = 3;
formation_offsets = [0, -20; -20, 0; -20, -20];
x_followers = repmat(x_leader, 1, num_followers);

% --- 🔹 Operating Points & Linearized Models ---
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
                   (v_nominal/L)*sec(delta_nominal)^2*dt 0;
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

% --- 🔹 Waypoints ---
waypoints = [30, 30; 50, 10; 20, 20; 0, 0; 30, 30; 50, 10; 20, 20; 0, 0];
num_waypoints = size(waypoints, 1);
waypoint_index = 1;

% --- 🔹 Logs ---
x_history_leader = x_leader;
x_history_followers = zeros(4, T_sim, num_followers);
u_history_followers = cell(1, num_followers);
mpc_state_leader = [];
mpc_state_followers = cell(1, num_followers);

% --- 🔹 Simulation Loop ---
for k = 1:T_sim
    if mod(k, 150) == 0 && waypoint_index < num_waypoints
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

    % --- Followers MPC Control ---
    for i = 1:num_followers
        offset = formation_offsets(i,:)';
        R = [cos(x_leader(3)), -sin(x_leader(3)); sin(x_leader(3)), cos(x_leader(3))];
        offset_global = R * offset;
        xref_follower = [x_leader(1:2) + offset_global; x_leader(3:4)];

        [~, theta_idx] = min(abs(theta_values - x_followers(3,i)));
        mpc_current = mpc_controllers{theta_idx};
        if isempty(mpc_state_followers{i})
            mpc_state_followers{i} = mpcstate(mpc_current);
        end
        u_follower = mpcmove(mpc_current, mpc_state_followers{i}, x_followers(:,i), xref_follower);
        x_followers(:,i) = A_models{theta_idx} * x_followers(:,i) + B_models{theta_idx} * u_follower;
        x_history_followers(:,k,i) = x_followers(:,i);
        u_history_followers{i} = [u_history_followers{i}; u_follower'];
    end
end

% --- 🔹 Plot Animation ---
figure;
hold on; grid on; axis equal;
xlim([-50 60]); ylim([-30 40]);
xlabel('X Position (m)'); ylabel('Y Position (m)');
title('Leader-Follower Formation Control (Dynamic Following)');

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
