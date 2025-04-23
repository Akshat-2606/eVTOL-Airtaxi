clear; clc; close all;

%% Simulation Parameters
dt = 0.02;
T_end = 20;
t = 0:dt:T_end;

g = 9.81;     % gravity
m = 1.5;      % UAV mass

%% Desired Trajectory Setup
x_ref = zeros(size(t));
z_ref = zeros(size(t));

for i = 1:length(t)
    if t(i) <= 5
        z_ref(i) = 5 * (t(i)/5);  % linear rise
        x_ref(i) = 0;
    else
        tau = (t(i)-5)/15;
        tau = min(tau, 1);

        P0 = [0, 5];
        P1 = [0, 10];
        P2 = [15, 10];
        P3 = [20, 10];

        B = (1 - tau)^3 * P0 + ...
            3 * (1 - tau)^2 * tau * P1 + ...
            3 * (1 - tau) * tau^2 * P2 + ...
            tau^3 * P3;

        x_ref(i) = B(1);
        z_ref(i) = B(2);
    end
end

%% LQR Controller Design
A = [0 1; 0 0];
B = [0; 1];

Q_z = diag([60, 20]);
R_z = 1;
K_z = lqr(A, B, Q_z, R_z);

Q_x = diag([60, 20]);
R_x = 1;
K_x = lqr(A, B, Q_x, R_x);

%% State Variables
z = 0; z_dot = 0;
x = 0; x_dot = 0;

z_log = zeros(size(t));
x_log = zeros(size(t));

%% Set up 3D animation
f = figure('Color', 'w');
ax = axes('XLim', [0 25], 'YLim', [-1 1], 'ZLim', [0 12]);
xlabel('X [m]'); ylabel('Y'); zlabel('Z [m]');
grid on; view(3); hold on;
title('UAV Trajectory Tracking');

plot3(x_ref, zeros(size(x_ref)), z_ref, 'b--', 'LineWidth', 1.5);
uav_dot = plot3(0, 0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
actual_path = plot3(nan, nan, nan, 'k-', 'LineWidth', 1.5);

err_display = uicontrol('Style','text',...
    'Position',[20 20 300 40],...
    'String','Error %: X = 0.00%, Z = 0.00%',...
    'FontSize', 11,...
    'BackgroundColor', 'w',...
    'HorizontalAlignment','left');

%% Simulation Loop
for i = 1:length(t)
    % Reference
    x_des = x_ref(i);
    z_des = z_ref(i);

    % States
    state_z = [z; z_dot];
    state_x = [x; x_dot];

    % Desired states
    des_z = [z_des; 0];
    des_x = [x_des; 0];

    % LQR Control Inputs
    u_z = -K_z * (state_z - des_z) + g;  % compensate gravity
    u_x = -K_x * (state_x - des_x);

    % Dynamics
    z_ddot = u_z - g;
    x_ddot = u_x;

    z_dot = z_dot + z_ddot * dt;
    z = z + z_dot * dt;

    x_dot = x_dot + x_ddot * dt;
    x = x + x_dot * dt;

    % Store trajectory
    z_log(i) = z;
    x_log(i) = x;

    % Compute error percentages
    x_err_percent = abs((x_des - x) / max(abs(x_des), 0.01)) * 100;
    z_err_percent = abs((z_des - z) / max(abs(z_des), 0.01)) * 100;

    % UI update
    set(err_display, 'String', ...
        sprintf('Error %%: X = %.2f%%, Z = %.2f%%', x_err_percent, z_err_percent));

    % Update animation
    set(uav_dot, 'XData', x, 'YData', 0, 'ZData', z);
    set(actual_path, 'XData', x_log(1:i), 'YData', zeros(1,i), 'ZData', z_log(1:i));
    drawnow;
end

%% Error Plots
error_x = x_ref - x_log;
error_z = z_ref - z_log;
error_squared = error_x.^2 + error_z.^2;

figure;
plot(t, error_squared, 'r', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Tracking Error^2');
title('Total Squared Tracking Error vs Time');
grid on;

figure;
plot(t, abs(error_x), 'b', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('X Error [m]');
title('X Error vs Time');
grid on;

figure;
plot(t, abs(error_z), 'g', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Z Error [m]');
title('Z Error vs Time');
grid on;
