clc; clear; close all;

% --- 🔹 Parameters ---
dt = 0.1;
T_sim = 800;
L = 2.5;
N = 20; M = 10;
rotate_formation = 1;

% --- 🔹 Leader Init ---
x_leader = [0; 0; 0; 0];
xref_leader = [30; 30; 0; 0];

% --- 🔹 3 Followers with fixed (global) square offsets ---
num_followers = 3;
formation_offsets = [0 -20; -20 0; -20 -20];  % Fixed in world frame
x_followers = repmat(x_leader, 1, num_followers);
v_integral = zeros(1, num_followers);

% --- 🔹 PID Gains ---
Kp_v = 14; Ki_v = 6; Kd_v = 0.2;
Kp_theta = 10;

% --- 🔹 Leader's MPC Model Setup ---
v_nominal = 10; delta_nominal = 0;
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
    B_models{j} = [0 0; 0 0;
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

% --- 🔹 Waypoints ---
waypoints = [30 30; 50 10; 20 20; 0 0; 30 30; 50 10; 20 20; 0 0];
waypoint_index = 1;

% --- 🔹 Logs ---
x_history_leader = x_leader;
x_history_followers = zeros(4, T_sim, num_followers);
mpc_state_leader = [];

% --- 🔹 Simulation Loop ---
for k = 1:T_sim
    if mod(k, 100) == 0 && waypoint_index < size(waypoints, 1)
        waypoint_index = waypoint_index + 1;
    end
    xref_leader = [waypoints(waypoint_index, :)'; 0; 0];

    [~, theta_idx] = min(abs(theta_values - x_leader(3)));
    mpc_current = mpc_controllers{theta_idx};
    if isempty(mpc_state_leader)
        mpc_state_leader = mpcstate(mpc_current);
    end
    u_leader = mpcmove(mpc_current, mpc_state_leader, x_leader, xref_leader);
    x_leader = A_models{theta_idx} * x_leader + B_models{theta_idx} * u_leader;
    x_history_leader = [x_history_leader, x_leader];

    for i = 1:num_followers
        R = [cos(x_leader(3)), -sin(x_leader(3));
             sin(x_leader(3)),  cos(x_leader(3))];
        if rotate_formation
            target_pos = x_leader(1:2) + R * formation_offsets(i,:)';
        else
            target_pos = x_leader(1:2) + formation_offsets(i,:)';
        end

        x_f = x_followers(1, i);
        y_f = x_followers(2, i);
        theta_f = x_followers(3, i);
        v_f = x_followers(4, i);

        vec_to_target = target_pos - [x_f; y_f];
        dist_to_target = norm(vec_to_target);
        v_desired = min(12, dist_to_target);
        v_error = v_desired - v_f;

        v_integral(i) = v_integral(i) + v_error * dt;
        v_integral(i) = min(max(v_integral(i), -10), 10);

        a_cmd = Kp_v * v_error + Ki_v * v_integral(i) - Kd_v * v_f;

        target_heading = atan2(vec_to_target(2), vec_to_target(1));
        heading_error = wrapToPi(target_heading - theta_f);

        delta_cmd = Kp_theta * heading_error;
        delta_cmd = max(min(delta_cmd, deg2rad(40)), -deg2rad(40));

        x_f = x_f + v_f * cos(theta_f) * dt;
        y_f = y_f + v_f * sin(theta_f) * dt;
        theta_f = theta_f + (v_f / L) * tan(delta_cmd) * dt;
        v_f = v_f + a_cmd * dt;

        x_followers(:, i) = [x_f; y_f; theta_f; v_f];
        x_history_followers(:, k, i) = x_followers(:, i);
    end
end

% --- 🔹 Animation ---
figure;
hold on; grid on; axis equal;
xlim([-50 60]); ylim([-30 40]);
xlabel('X Position (m)'); ylabel('Y Position (m)');
title('Formation Control - Upright Square (No Rotation)');

h_cross = plot(NaN, NaN, 'rx', 'MarkerSize', 12, 'LineWidth', 2);
h_leader = plot(NaN, NaN, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
h_followers = gobjects(1, num_followers);
for i = 1:num_followers
    h_followers(i) = plot(NaN, NaN, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
end

visual_lag_offset = 6;

for k = 1:T_sim
    set(h_cross, 'XData', x_history_leader(1,k), ...
                 'YData', x_history_leader(2,k));
    display_idx = max(k - visual_lag_offset, 1);
    set(h_leader, 'XData', x_history_leader(1, display_idx), ...
                  'YData', x_history_leader(2, display_idx));

    for i = 1:num_followers
        set(h_followers(i), 'XData', x_history_followers(1,k,i), ...
                            'YData', x_history_followers(2,k,i));
    end
    drawnow;
    pause(0.05);
end

disp('Simulation Complete!');
