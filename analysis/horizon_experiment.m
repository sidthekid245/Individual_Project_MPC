clc; clear; close all;

% Experiment Setup
prediction_horizons = [5, 10, 15, 20];
control_horizons = [2, 5, 10];
dt = 0.1;
L = 2.5;
T_sim = 100;
xref = [20; 20; 0; 0];
v_nominal = 10;
delta_nominal = 0;
theta_values = linspace(-pi, pi, 120);

% Precompute models
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
end

% Results Table
results = [];

for N = prediction_horizons
    for M = control_horizons
        % Build MPC controllers
        for j = 1:length(theta_values)
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

        % Run simulation
        x = [0; 0; 0; 0];
        x_history = x;
        u_history = [];

        mpc_state = [];

        for k = 1:T_sim
            [~, theta_idx] = min(abs(theta_values - x(3)));
            mpc_current = mpc_controllers{theta_idx};
            if isempty(mpc_state) || ~isa(mpc_state, 'mpcstate')
                mpc_state = mpcstate(mpc_current);
            end
            u_mpc = mpcmove(mpc_current, mpc_state, x, xref);
            u_mpc = u_mpc(:);
            x = A_models{theta_idx} * x + B_models{theta_idx} * u_mpc;
            x_history = [x_history, x];
            u_history = [u_history; u_mpc'];
        end

        % Metrics
        pos_error = vecnorm(x_history(1:2,:) - xref(1:2));
        rmse = sqrt(mean(pos_error.^2));
        ss_error = pos_error(end);
        control_effort = sum(vecnorm(u_history, 2, 2));
        smoothness = sum(vecnorm(diff(u_history), 2, 2));
        peak_input = max(max(abs(u_history)));

        % Settling time (within 5% of target position)
        within_threshold = pos_error < 0.05 * norm(xref(1:2));
        settled_idx = find(cumsum(~within_threshold) == 0, 1, 'last');
        if isempty(settled_idx)
            settling_time = T_sim * dt;
        else
            settling_time = settled_idx * dt;
        end

        % Store result
        results = [results; N, M, rmse, ss_error, control_effort, ...
                   smoothness, peak_input, settling_time];
    end
end

% Table output
T = array2table(results, ...
    'VariableNames', {'PredictionHorizon', 'ControlHorizon', 'RMSE', ...
                      'SteadyStateError', 'ControlEffort', 'Smoothness', ...
                      'PeakInput', 'SettlingTime'});

disp(T);

% --- 🔹 RMSE vs Prediction Horizon ---
figure;
[G_pred, pred_vals] = findgroups(T.PredictionHorizon);
avg_rmse = splitapply(@mean, T.RMSE, G_pred);
bar(pred_vals, avg_rmse);
xlabel('Prediction Horizon');
ylabel('Average RMSE');
title('RMSE vs Prediction Horizon');
grid on;

% --- 🔹 Control Effort vs Control Horizon ---
figure;
[G_ctrl, ctrl_vals] = findgroups(T.ControlHorizon);
avg_effort = splitapply(@mean, T.ControlEffort, G_ctrl);
bar(ctrl_vals, avg_effort);
xlabel('Control Horizon');
ylabel('Average Control Effort');
title('Control Effort vs Control Horizon');
grid on;
