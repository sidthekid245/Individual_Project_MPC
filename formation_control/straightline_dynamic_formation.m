clc; clear; close all;

% --- 🔹 Simulation Parameters ---
dt = 0.1;
T_sim = 500;
L = 2.5;

% --- 🔹 Leader (Open-loop straight line) ---
v_leader_const = 5;
delta_leader_const = deg2rad(5);
x_leader = [0; 0; deg2rad(0); v_leader_const];

% --- 🔹 One Follower at (-10, 0) ---
formation_offset = [-10; 0];
x_follower = x_leader;
v_integral = 0;

% --- 🔹 Logs ---
x_history_leader = x_leader;
x_history_follower = zeros(4, T_sim);

% --- 🔹 PID Gains ---
Kp_v = 14;
Ki_v = 6;
Kd_v = 0.2;
Kp_theta = 10;

% --- 🔹 Simulation Loop ---
for k = 1:T_sim
    % --- Leader motion (open-loop KBM) ---
    x = x_leader(1);
    y = x_leader(2);
    theta = x_leader(3);
    v = x_leader(4);

    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    theta = theta + (v / L) * tan(delta_leader_const) * dt;

    x_leader = [x; y; theta; v];
    x_history_leader = [x_history_leader, x_leader];

    % --- Follower target position (no feedforward) ---
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    target_pos = x_leader(1:2) + R * formation_offset;

    % --- Follower motion ---
    x_f = x_follower(1);
    y_f = x_follower(2);
    theta_f = x_follower(3);
    v_f = x_follower(4);

    vec_to_target = target_pos - [x_f; y_f];
    dist_to_target = norm(vec_to_target);
    v_desired = min(8, dist_to_target);  % No v_ff
    v_error = v_desired - v_f;

    v_integral = v_integral + v_error * dt;
    v_integral = min(max(v_integral, -10), 10);

    a_cmd = Kp_v * v_error + Ki_v * v_integral - Kd_v * v_f;

    target_heading = atan2(vec_to_target(2), vec_to_target(1));
    heading_error = wrapToPi(target_heading - theta_f);

    delta_cmd = Kp_theta * heading_error;
    delta_cmd = max(min(delta_cmd, deg2rad(40)), -deg2rad(40));

    x_f = x_f + v_f * cos(theta_f) * dt;
    y_f = y_f + v_f * sin(theta_f) * dt;
    theta_f = theta_f + (v_f / L) * tan(delta_cmd) * dt;
    v_f = v_f + a_cmd * dt;

    x_follower = [x_f; y_f; theta_f; v_f];
    x_history_follower(:, k) = x_follower;
end

% --- 🔹 Animation ---
figure;
hold on; grid on; axis equal;
xlim([-80 80]); ylim([-20 80]);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Straight Line Test - Clean Baseline');

h_leader = plot(NaN, NaN, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
h_follower = plot(NaN, NaN, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');

for k = 1:T_sim
    set(h_leader, 'XData', x_history_leader(1,k), 'YData', x_history_leader(2,k));
    set(h_follower, 'XData', x_history_follower(1,k), ...
                    'YData', x_history_follower(2,k));
    drawnow;
    pause(0.05);
end

disp('Straight Line Test Complete!');
