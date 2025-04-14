clc; clear; close all;

% --- 🔹 Parameters ---
dt = 0.1;
T_sim = 400;
L = 2.5;
rotate_formation = 1;

% --- 🔹 Leader Initialization ---
x_leader = [0; 0; 0; 5];  % [x; y; theta; velocity]
segment_time = 100;
delta_leader = [0, pi/2, pi, -pi/2];
current_segment = 1;

% --- 🔹 Formation Setup ---
num_followers = 3;
formation_offsets = [0 -20; -20 0; -20 -20];
x_followers = repmat(x_leader, 1, num_followers);
v_integral = zeros(1, num_followers);

% --- 🔹 PID Gains ---
Kp_v = 14; Ki_v = 6; Kd_v = 0.2;
Kp_theta = 10;

% --- 🔹 Logs ---
x_history_leader = x_leader;
x_history_followers = zeros(4, T_sim, num_followers);

% --- 🔹 Simulation Loop ---
for k = 1:T_sim
    if mod(k-1, segment_time) == 0 && k > 1
        current_segment = current_segment + 1;
        if current_segment > length(delta_leader)
            current_segment = length(delta_leader);
        end
    end

    x_leader(3) = delta_leader(current_segment);
    x_leader(1) = x_leader(1) + x_leader(4) * cos(x_leader(3)) * dt;
    x_leader(2) = x_leader(2) + x_leader(4) * sin(x_leader(3)) * dt;
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

% --- 🔹 Animation Setup ---
visual_lag_offset = 8;

figure;
hold on; grid on; axis equal;
xlim([-30 80]); ylim([-30 80]);
xlabel('X (m)'); ylabel('Y (m)');
title('Leader Square Path - PID Followers');

h_leader = plot(NaN, NaN, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');  % Lagged position
h_cross = plot(NaN, NaN, 'rx', 'MarkerSize', 10, 'LineWidth', 2);  % True leader position
h_followers = gobjects(1, num_followers);

for i = 1:num_followers
    h_followers(i) = plot(NaN, NaN, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
end

legend([h_leader, h_cross, h_followers(1)], ...
       {'Leader (Plotted w/ Lag)', 'Leader Actual Position', 'Followers'}, ...
       'Location', 'northwest');

% --- 🔹 Animation Loop ---
for k = 1:T_sim
    display_idx = max(k - visual_lag_offset, 1);
    set(h_leader, 'XData', x_history_leader(1, display_idx), ...
                  'YData', x_history_leader(2, display_idx));
    set(h_cross, 'XData', x_history_leader(1,k), ...
                 'YData', x_history_leader(2,k));

    for i = 1:num_followers
        set(h_followers(i), 'XData', x_history_followers(1,k,i), ...
                            'YData', x_history_followers(2,k,i));
    end
    drawnow;
    pause(0.01);
end

disp('Simulation Complete!');
