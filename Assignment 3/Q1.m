%% Assignment 3 - Question 1: State-Space Design, Pole Placement & Varying K
clear; clc; close all;

%% 1. System Definition (Question 1a & 1b)
Ts = 0.01;          % Sampling period [s]

% Discrete-time state-space matrices (Forward Euler)
A = 1;
B = Ts;
C = -1;             % Sensor measures positive distance while cart is at x <= 0
D = 0;

%% 2. Pole Placement & Reference Scaling for Design Choice (Question 1c & 1d)
p_c = 0.98;         % Desired closed-loop pole inside the unit circle
K = (1 - p_c) / Ts; % State feedback gain K = 2 [1/s]
Nr = -K;            % Feedforward scaling for unity DC gain (y_ss = r_ss)

fprintf('=== ASSIGNMENT 3 - QUESTION 1 DESIGN ===\n');
fprintf('Desired pole p_c  : %.4f\n', p_c);
fprintf('Feedback gain K   : %.4f [1/s]\n', K);
fprintf('Feedforward Nr    : %.4f [1/s]\n\n', Nr);

% Closed-loop system for the chosen design (K = 2)
sys_cl = ss(A - B*K, B*Nr, C, D, Ts);

%% 3. Simulation for Selected Design (K = 2)
t_final = 3.0;                  % Simulation duration [s]
t = 0:Ts:t_final;
N = length(t);

x0 = -0.25;                     % Initial condition: cart at 0.25 m from wall
r = 0.15 * ones(N, 1);          % Reference distance: 0.15 m

[y, ~, x] = lsim(sys_cl, r, t, x0);
v = -K * x + Nr * r;            % Commanded velocity control signal v[k]

%% 4. Varying K Analysis & Step Response Comparison (Question 1c requirement)
% Different regimes: overdamped, optimal (K=2), deadbeat (1/Ts = 100), oscillatory
K_values = [0.5, 2.0, 10, 100, 150]; 
colors = lines(length(K_values));

figure('Units', 'centimeters', 'Position', [5 5 16 10]);
hold on; grid on;

plot(t, r, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference r[k]');

for i = 1:length(K_values)
    Ki = K_values(i);
    Nri = -Ki;
    p_i = 1 - Ts * Ki;
    
    % Closed-loop system for varying K
    sys_temp = ss(A - B*Ki, B*Nri, C, D, Ts);
    [y_temp, ~] = lsim(sys_temp, r, t, x0);
    
    label_str = sprintf('K = %.1f s^{-1} (p_c = %.2f)', Ki, p_i);
    plot(t, y_temp, 'Color', colors(i,:), 'LineWidth', 1.3, 'DisplayName', label_str);
end

xlabel('Time [s]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Measured Distance y[k] [m]', 'FontSize', 11, 'FontWeight', 'bold');
title('Closed-Loop Response for Varying State Feedback Gains K', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'Northeast');
ylim([0 0.35]);

% Export comparison figure for LaTeX
if ~exist('Images', 'dir'), mkdir('Images'); end
exportgraphics(gcf, 'Images/Ass3_Q1_K_comparison.png', 'Resolution', 300);

%% 5. Detailed Plots for Selected Design (K = 2)
figure('Units', 'centimeters', 'Position', [5 5 16 12]);

subplot(2,1,1);
plot(t, r, 'k--', 'LineWidth', 1.5); hold on;
plot(t, y, 'b-', 'LineWidth', 1.5);
grid on;
ylabel('Distance y[k] [m]', 'FontSize', 11, 'FontWeight', 'bold');
title(sprintf('Closed-Loop Position Tracking (K = %.1f s^{-1}, p_c = %.2f)', K, p_c), 'FontSize', 12, 'FontWeight', 'bold');
legend('Reference', 'Simulated Distance', 'Location', 'Northeast');

subplot(2,1,2);
plot(t, v, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Velocity Input v[k] [m/s]', 'FontSize', 11, 'FontWeight', 'bold');
title('Commanded Forward Velocity v[k]', 'FontSize', 12, 'FontWeight', 'bold');

exportgraphics(gcf, 'Images/Ass3_Q1_step_response.png', 'Resolution', 300);

%% Pole-Zero Map for Varying K (Question 1c requirement)
Ts = 0.01;
A = 1; B = Ts; C = -1; D = 0;

K_values = [0.5, 2.0, 10, 100, 150];
colors = lines(length(K_values));

figure('Units', 'centimeters', 'Position', [5 5 14 14]);
hold on; grid on;

% Teken de eenheidscirkel voor stabiliteitsreferentie
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'k--', 'LineWidth', 1.2, 'DisplayName', 'Unit Circle');

% Plot de polen voor elke waarde van K
for i = 1:length(K_values)
    Ki = K_values(i);
    p_i = 1 - Ts * Ki; % Gesloten-loop pool in het z-domein
    
    label_str = sprintf('K = %.1f s^{-1} (p_c = %.2f)', Ki, p_i);
    plot(p_i, 0, 'x', 'MarkerSize', 10, 'LineWidth', 2, 'Color', colors(i,:), 'DisplayName', label_str);
end

axis([-1.2 1.2 -0.8 0.8]);
xlabel('Real Axis (Re)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Imaginary Axis (Im)', 'FontSize', 11, 'FontWeight', 'bold');
title('Pole Migration in the z-Plane for Varying K', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'Southwest');

% Opslaan voor LaTeX
if ~exist('Images', 'dir'), mkdir('Images'); end
exportgraphics(gcf, 'Images/pzmap_K.png', 'Resolution', 300);