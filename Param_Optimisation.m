clear; clc;

%% Parameters
dt = 0.02;               % Time step (seconds)
T_end = 20;              % Total simulation time (seconds)
t = 0:dt:T_end;          % Time vector

g = 9.81;                % Gravity (m/s^2)
m = 1.5;                 % UAV mass (kg)

% Desired altitude (for simplicity, we aim for a straight line to 5 meters)
z_ref = 5 * ones(size(t));  % Reference altitude: Constant 5 meters

%% Objective Function for PID Tuning (to minimize overshoot, steady-state error, and settling time)
function J = pid_optimization(Kp, Ki, Kd, t, z_ref, dt, g)
    % Initialize state variables and errors
    z = 0; z_dot = 0;
    int_err_z = 0; prev_err_z = 0;
    max_integral = 10;  % Max limit for integral windup
    
    z_log = [];   % For logging altitude trajectory

    for i = 1:length(t)
        % Calculate error
        err_z = z_ref(i) - z;
        int_err_z = int_err_z + err_z * dt;
        
        % Limit integral term to prevent windup
        int_err_z = max(min(int_err_z, max_integral), -max_integral);
        
        der_err_z = (err_z - prev_err_z) / dt;
        prev_err_z = err_z;

        % PID Control law
        acc_z = Kp * err_z + Ki * int_err_z + Kd * der_err_z;

        % Apply dynamics
        z_ddot = acc_z - g;
        z_dot = z_dot + z_ddot * dt;
        z = z + z_dot * dt;
        
        % Log altitude
        z_log = [z_log, z];
    end

    % Objective function: minimize overshoot, steady-state error, and settling time
    steady_state_error = abs(z_log(end) - z_ref(end));  % Steady-state error at the end
    overshoot = max(z_log) - max(z_ref);
    settling_time = find(abs(z_log - max(z_ref)) < 0.02, 1, 'last') * dt;

    if isempty(overshoot) || isempty(settling_time)
        J = inf;  % Invalid solution
    else
        % Cost: overshoot + settling time + steady-state error
        J = overshoot + settling_time + steady_state_error;
    end
end

%% PSO Optimization for PID Gains
% Set up the optimization problem
options = optimoptions('particleswarm', 'SwarmSize', 50, 'MaxIterations', 50, 'Display', 'iter');

% Define bounds for the PID gains
lb = [0, 0, 0];     % Lower bounds for PID gains
ub = [20, 2, 5];    % Upper bounds for PID gains

% Run Particle Swarm Optimization (PSO)
[param_opt, J_opt] = particleswarm(@(x) pid_optimization(x(1), x(2), x(3), t, z_ref, dt, g), 3, lb, ub, options);

% Display optimized PID gains and objective function value
fprintf('Optimized Kp: %.4f, Ki: %.4f, Kd: %.4f\n', param_opt(1), param_opt(2), param_opt(3));
fprintf('Minimum Objective Function Value (Overshoot + Settling Time + Steady-State Error): %.4f\n', J_opt);
