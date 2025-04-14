clear; clc; close all;

% Define Vehicle Parameters
L = 2.5; % Wheelbase (meters)
v0 = 5; % Nominal velocity (m/s)
theta0 = 0; % Nominal heading angle (radians)
delta0 = 0; % Nominal steering angle (radians)
Ts = 0.1; % Sampling time (seconds)

% Define A and B matrices (Linearized System)
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

sysc = ss(A, B, C, D); % Continuous state-space model
sysd = c2d(sysc, Ts, 'zoh'); % Convert to discrete-time

Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;
Dd = sysd.D;

mpcObj = mpc(sysd);

mpcObj.PredictionHorizon = 10;
mpcObj.ControlHorizon = 3;

mpcObj.ManipulatedVariables(1).Min = -2;  % Steering angle (rad)
mpcObj.ManipulatedVariables(1).Max = 2;

mpcObj.ManipulatedVariables(2).Min = -2;  % Acceleration (m/s^2)
mpcObj.ManipulatedVariables(2).Max = 2;

mpcObj.OutputVariables(4).Min = 0; % Velocity should not be negative
mpcObj.OutputVariables(4).Max = 15; % Maximum velocity

mpcObj.Weights.OutputVariables = [1 1 1 0]; % Prioritize position and heading
mpcObj.Weights.ManipulatedVariables = [0.1 0.1]; % Penalize input effort
mpcObj.Weights.ManipulatedVariablesRate = [0.01 0.01]; % Penalize input changes

x0 = [0; 0; 0; v0]; % Start at (0,0) with nominal velocity
ref = [2; 20; pi/4; v0]; % Desired final state

Tsim = 50; % Total simulation time
x = x0;
u = [0; 0]; % Initial control inputs
X_log = x0'; % Log state history
U_log = u'; % Log control inputs

mpcState = mpcstate(mpcObj); % Initialize the MPC state object

for k = 1:Tsim
    % Compute optimal control action
    u = mpcmove(mpcObj, mpcState, x, ref);
    
    % Apply control and update state
    x = Ad*x + Bd*u;
    
    % Log data
    X_log = [X_log; x'];
    U_log = [U_log; u'];
end

figure;
plot(X_log(:,1), X_log(:,2), 'b-o'); hold on;
plot(ref(1), ref(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('MPC-based Agent Trajectory');
grid on;
legend('Path', 'Target');

figure;
subplot(2,1,1);
plot(U_log(:,1), 'r');
ylabel('Steering Angle (rad)');
title('Improved Control Inputs');
grid on;

subplot(2,1,2);
plot(U_log(:,2), 'b');
ylabel('Acceleration (m/s²)');
xlabel('Time Steps');
grid on;
