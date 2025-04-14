clc; clear; close all;

% --- 🔹 Simulation Parameters ---
dt = 0.1;       % Time step (s)
N = 20;         % Prediction Horizon
M = 10;         % Control Horizon
L = 2.5;        % Wheelbase (m)
T_sim = 800;    % Simulation time steps

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
v_nominal = 10;  
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
    mpc_controllers{j}.Weights.ManipulatedVariables = [0.01 0.01];
    mpc_controllers{j}.Weights.ManipulatedVariablesRate = [0.005 0.005];
    mpc_controllers{j}.Weights.OutputVariables = [0.5 0.5 0 0.1];
    mpc_controllers{j}.MV(1).Min = -deg2rad(30);
    mpc_controllers{j}.MV(1).Max = deg2rad(30);
    mpc_controllers{j}.MV(2).Min = -4;
    mpc_controllers{j}.MV(2).Max = 4;
end

% --- 🔹 Initialize Logs ---
x_history_leader = x_leader;
x_history_followers = zeros(4, T_sim, num_followers);
u_history_leader = [];
u_history_followers = cell(1, num_followers);

% --- 🔹 Initialize MPC State (Before Loop) ---
mpc_state_leader = [];
mpc_state_followers = cell(1, num_followers);

% --- 🔹 Define Leader's Waypoints (Target Positions) ---
waypoints = [30, 30;
             50, 10;
             20, 20;
              0,  0;
             30, 30;
             50, 10;
             20, 20;
              0,  0];
num_waypoints = size(waypoints, 1);
waypoint_index = 1; % Start from the first waypoint

% --- 🔹 Simulation Loop ---
for k = 1:T_sim
    % --- Switch Leader's Target Every 100 Steps ---
    if mod(k, 100) == 0 && waypoint_index < num_waypoints
        waypoint_index = waypoint_index + 1;
    end
    xref_leader = [waypoints(waypoint_index, :)'; 0; 0];  % Update leader target

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

% --- 🔹 Initialize Waypoint Index ---
waypoint_index = 1; % Start with the first waypoint

% --- 🔹 Initialize Figure ---
figure;
hold on;
grid on;
axis equal;
xlim([-50 60]);
ylim([-30 40]);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Leader-Follower Formation Control with MPC');

% --- 🔹 Initialize Plots ---
h_leader = plot(NaN, NaN, 'r--', 'LineWidth', 1.5); % Leader Path
h_leader_marker = plot(NaN, NaN, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % Leader Marker

h_followers = gobjects(1, num_followers); % Followers' Paths
h_follower_markers = gobjects(1, num_followers); % Followers' Markers

for i = 1:num_followers
    h_followers(i) = plot(NaN, NaN, 'b--', 'LineWidth', 1.5); % Follower Paths
    h_follower_markers(i) = plot(NaN, NaN, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b'); % Follower Markers
end

% --- 🔹 Initialize Target Points ---
h_target_leader = plot(NaN, NaN, 'rx', 'MarkerSize', 12, 'LineWidth', 2); % Target for Leader
h_target_followers = gobjects(1, num_followers); % Targets for Followers

for i = 1:num_followers
    h_target_followers(i) = plot(NaN, NaN, 'gx', 'MarkerSize', 12, 'LineWidth', 2);
end

% --- 🔹 Animate the Motion Without Path Traces ---
for k = 1:T_sim
    % --- 🔹 Check if We Need to Update Target Positions ---
    if mod(k, 100) == 1  
        % Hide previous targets
        set(h_target_leader, 'XData', NaN, 'YData', NaN);
        for i = 1:num_followers
            set(h_target_followers(i), 'XData', NaN, 'YData', NaN);
        end

        % Update to the next waypoint if available
        if waypoint_index <= num_waypoints
            % Set new target position for the leader
            set(h_target_leader, 'XData', waypoints(waypoint_index,1), 'YData', waypoints(waypoint_index,2));

            % Set new target positions for followers
            for i = 1:num_followers
                xref_follower_final = waypoints(waypoint_index, 1:2) + formation_offsets(i,:);
                set(h_target_followers(i), 'XData', xref_follower_final(1), 'YData', xref_follower_final(2));
            end

            % Move to the next waypoint
            waypoint_index = waypoint_index + 1;
        end
    end

    % --- 🔹 Update Leader's Current Position Only ---
    set(h_leader_marker, 'XData', x_history_leader(1,k), 'YData', x_history_leader(2,k));

    % --- 🔹 Update Followers' Current Positions Only ---
    for i = 1:num_followers
        set(h_follower_markers(i), 'XData', x_history_followers(1,k,i), ...
                                   'YData', x_history_followers(2,k,i));
    end

    % --- 🔹 Refresh Plot ---
    drawnow;
    pause(0.05); % Adjust animation speed
end

legend('Leader Path', 'Follower Paths', 'Leader Target', 'Follower Targets');
disp('Animation Complete!');