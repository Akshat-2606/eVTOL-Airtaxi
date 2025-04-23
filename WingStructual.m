% Wing Parameters
b = 2;                % semi-span [m]
c = 0.12;             % chord [m]
t = 0.015;            % thickness estimate [m]
E = 70e9;             % Young's Modulus for Aluminum [Pa]
rho_air = 1.225;      % air density [kg/m^3]
V = 70;               % cruise speed [m/s]
Cl = 1.2;             % lift coefficient

% Structural Properties
I = (c * t^3) / 12;   % Moment of inertia [m^4]

% Discretization
N = 500;              % Higher resolution for detailed plots
y = linspace(0, b, N); 
dy = y(2) - y(1);

% Elliptical lift distribution (per unit span)
S = 2 * b * c;        
L_total = 0.5 * rho_air * V^2 * S * Cl;  
q0 = (4 * L_total) / (pi * b);           
q = q0 * (1 - (y / b).^2);               

% Shear force V(y): integrate load q(y) from tip to root
V = cumtrapz(flip(y), flip(q));
V = flip(V);  % flip back to match original order

% Bending moment M(y): integrate shear from tip to root
M = cumtrapz(flip(y), flip(V));
M = flip(M);  % flip back to match original order

% Deflection by double integration of moment
theta = cumtrapz(y, M / (E * I));
w = cumtrapz(y, theta);

% Normalize deflection to start at 0 at root
w = w - w(1);

% === Plots ===
figure;

subplot(2,2,1)
plot(y, q, 'r', 'LineWidth', 2);
xlabel('Spanwise Position y [m]');
ylabel('Lift Load q(y) [N/m]');
title('Distributed Lift Load Profile');
grid on;
set(gca, 'FontSize', 12);
xlim([0 b]);
ylim([0 max(q)*1.1]);
xtickformat('%.1f');
text(b*0.7, max(q)*0.7, 'Elliptical Distribution', 'FontSize', 12, 'Color', 'red');

subplot(2,2,2)
plot(y, V, 'g', 'LineWidth', 2);
xlabel('Spanwise Position y [m]');
ylabel('Shear Force V(y) [N]');
title('Shear Force Distribution');
grid on;
set(gca, 'FontSize', 12);
xlim([0 b]);
ylim([min(V)*1.1, max(V)*1.1]);
xtickformat('%.1f');
text(b*0.5, max(V)*0.7, 'Shear Force Variation', 'FontSize', 12, 'Color', 'green');

subplot(2,2,3)
plot(y, M, 'm', 'LineWidth', 2);
xlabel('Spanwise Position y [m]');
ylabel('Bending Moment M(y) [N·m]');
title('Bending Moment Distribution');
grid on;
set(gca, 'FontSize', 12);
xlim([0 b]);
ylim([min(M)*1.1, max(M)*1.1]);
xtickformat('%.1f');
text(b*0.5, max(M)*0.7, 'Max Moment at Root', 'FontSize', 12, 'Color', 'magenta');

subplot(2,2,4)
plot(y, w, 'b', 'LineWidth', 2);
xlabel('Spanwise Position y [m]');
ylabel('Deflection w(y) [m]');
title('Wing Deflection under Load');
grid on;
set(gca, 'FontSize', 12);
xlim([0 b]);
ylim([min(w)*1.1, max(w)*1.1]);
xtickformat('%.1f');
text(b*0.3, max(w)*0.7, 'Maximum Deflection at Tip', 'FontSize', 12, 'Color', 'blue');
