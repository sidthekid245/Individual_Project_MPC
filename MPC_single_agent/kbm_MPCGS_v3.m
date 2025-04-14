clear; clc; close all;

% Define Vehicle Parameters
L = 2.5; % Wheelbase (meters)
v0 = 5; % Nominal velocity (m/s)
theta0 = 0; % Nominal heading angle (radians)
delta0 = 0; % Nominal steering angle (radians)
Ts = 0.1; % Sampling time (seconds)

% Initial Conditions
x0 = [0; 0; 0; v0]; % Start at (0,0) with nominal velocity
ref = [2; 20; pi/4; v0]; % Desired final state

% Define Initial Linearized System
A = [0, 0, -v0*sin(theta0), cos(theta0);
     0, 0, v0*cos(theta0), sin(theta0);
     0, 0, 0, (1/L)*tan(delta0);
     0, 0, 0, 0];

B = [0, 0;
     0, 0;
     (v0/L) * sec(delta0)^2, 0;
     0, 1];

C = eye(4); % Full state measurement
D = zeros(4,2); % No direct feedthrough

% Convert to discrete-time model (since MPC requires it)
sysc = ss(A, B, C, D);
sysd = c2d(sysc, Ts, 'zoh'); 

Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;
Dd = sysd.D;

% Initialize MPC Controller
mpcObj = mpc(sysd);
mpcObj.Ts = Ts;
mpcObj.PredictionHorizon = 20;
mpcObj.ControlHorizon = 15;

% Constraints
mpcObj.ManipulatedVariables(1).Min = -2.7;  
mpcObj.ManipulatedVariables(1).Max = 2.7;
mpcObj.ManipulatedVariables(2).Min = -5;  
mpcObj.ManipulatedVariables(2).Max = 5;
mpcObj.OutputVariables(4).Min = 0; 
mpcObj.OutputVariables(4).Max = 15;

% ✅ Smoother Control Input Changes
mpcObj.Weights.OutputVariables = [550 400 5 35]; 
mpcObj.Weights.ManipulatedVariables = [0.02 0.04]; 
mpcObj.Weights.ManipulatedVariablesRate = [0.00001 0.00001]; % ✅ Less aggressive control changes

% Initialize MPC State
mpcState = mpcstate(mpcObj);

% Simulating MPC
Tsim = 100; % Total simulation time
x = x0;
X_log = x0';
U_log = [0 0];

for k = 1:Tsim
    % Ensure MPC State is Consistent
    if k == 1
        mpcState = mpcstate(mpcObj);
    end

    % ✅ Smooth Velocity Reduction Near Target
    if abs(x(1) - ref(1)) < 1 && abs(x(2) - ref(2)) < 1
        ref(4) = max(0, ref(4) * 0.95); % ✅ Reduce velocity smoothly instead of step-wise
    end

    % ✅ Adaptive Linearization Only When Needed
    if abs(x(3) - theta0) > 0.08 || abs(x(4) - v0) > 0.3 || abs(U_log(end,1) - delta0) > 0.08  
        
        theta0 = x(3); % Update nominal heading angle
        v0 = x(4); % Update nominal velocity
        
        if k > 1
            delta0 = U_log(end,1); % Use last steering angle
        else
            delta0 = 0; % Initial assumption
        end

        % Recompute linearized A, B matrices
        A_new = [0, 0, -v0*sin(theta0), cos(theta0);
                 0, 0, v0*cos(theta0), sin(theta0);
                 0, 0, 0, (1/L)*tan(delta0);
                 0, 0, 0, 0];

        B_new = [0, 0;
                 0, 0;
                 (v0/L) * sec(delta0)^2, 0;
                 0, 1];

        % Convert to discrete-time
        sysd_new = c2d(ss(A_new, B_new, C, D), Ts, 'zoh');
        Ad_new = sysd_new.A;
        Bd_new = sysd_new.B;

        % Blend models for stability
        Ad = 0.95 * Ad + 0.05 * Ad_new; % ✅ More gradual blending (95%-5%)
        Bd = 0.95 * Bd + 0.05 * Bd_new;

        % ✅ Fully Update MPC Model 
        set(mpcObj, 'Model', ss(Ad, Bd, Cd, Dd, Ts));
    end 

    % Compute optimal control action
    u = mpcmove(mpcObj, mpcState, x, ref);
    
    % Apply control and update state
    x = Ad*x + Bd*u;
    
    % Log Data
    X_log = [X_log; x'];
    U_log = [U_log; u'];
end

% ✅ Plot Trajectory
figure;
plot(X_log(:,1), X_log(:,2), 'b-o'); hold on;
plot(ref(1), ref(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('MPC-Based Trajectory Tracking with Adaptive Linearization (Improved Smoothness)');
grid on;
legend('MAV Path', 'Target');

% ✅ Plot Control Inputs
figure;
subplot(2,1,1);
plot(U_log(:,1), 'r');
ylabel('Steering Angle (rad)');
title('Control Inputs');
grid on;

subplot(2,1,2);
plot(U_log(:,2), 'b');
ylabel('Acceleration (m/s²)');
xlabel('Time Steps');
grid on;
