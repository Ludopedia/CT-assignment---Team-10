%% Assignment 3 - Question 1: State Feedback & State Estimator Design
clear; clc; close all;

%% 1. System Definition (Question 1a & 1b)
Ts = 0.01;          % Sampling period [s]
A = 1;              % Discrete state matrix [-]
B = Ts;             % Discrete input matrix [s]
C = -1;             % Measurement matrix (sensor measures -x > 0) [-]
D = 0;              % Direct feedthrough matrix [s]

%% 2. Controller & Estimator Design via Pole Placement (Question 1c & 1d)
% Selected controller pole and gain K
p_c_design = 0.98;                      % Desired closed-loop controller pole [-]
K_design = place(A, B, p_c_design);     % K = (1 - p_c) / Ts = 2.0 [1/s]

% Selected estimator pole (5x faster in continuous time) and gain L
p_c_cont = log(p_c_design) / Ts;        % Equivalent continuous pole [1/s]
p_e_cont = 5 * p_c_cont;                % 5x faster estimator pole [1/s]
p_e_design = exp(p_e_cont * Ts);        % Discrete estimator pole (~0.90) [-]
L_design = place(A', C', p_e_design)';  % Prediction estimator gain L = p_e - 1 = -0.10 [-]

fprintf('=== ASSIGNMENT 3 - QUESTION 1 DESIGN RESULTS ===\n');
fprintf('Controller pole p_c : %.4f [-]  --> Gain K = %.4f [1/s]\n', p_c_design, K_design);
fprintf('Estimator pole p_e  : %.4f [-]  --> Gain L = %.4f [-]\n\n', p_e_design, L_design);

%% 3. Pole-Zero Map & Step Response for Varying K (Question 1c)
K_vals = [0.5, 2.0, 20, 100, 150];      % Varying K values [1/s]
line_styles = {'-', '--', ':', '-.', '-'};
markers = {'o', 's', '^', 'd', 'x'};

% Figure 1: Pole-Zero Map 
figure('Units', 'centimeters', 'Position', [5 5 12 11]);
hold on; grid on; box on;

% Draw unit circle
theta = linspace(0, 2*pi, 200);
plot(cos(theta), sin(theta), 'k:', 'LineWidth', 1.2, 'DisplayName', 'Unit circle');
xline(0, 'k-', 'LineWidth', 0.5, 'HandleVisibility', 'off');
yline(0, 'k-', 'LineWidth', 0.5, 'HandleVisibility', 'off');

% Plot pole migration for varying K
for i = 1:length(K_vals)
    Ki = K_vals(i);
    pi_val = 1 - Ts * Ki;
    lbl = sprintf('K = %.1f s^{-1} (p_c = %.2f)', Ki, pi_val);
    plot(pi_val, 0, 'Color', 'k', 'LineStyle', 'none', 'Marker', markers{i}, ...
         'MarkerSize', 10, 'LineWidth', 1.5, 'DisplayName', lbl);
end

% Add arrow showing direction of increasing K
annotation('textarrow', [0.78 0.35], [0.56 0.56], 'String', 'Increasing K', 'FontSize', 20);

xlim([-1.2 1.2]); ylim([-1.1 1.1]); axis equal;
xlabel('Real axis (Re)', 'FontSize', 30);
ylabel('Imaginary axis (Im)', 'FontSize', 30);
title('Closed-loop pole migration in z-plane', 'FontSize', 30, 'FontWeight', 'bold');
legend('Location', 'Southwest', 'FontSize', 20);

% Figure 2: Closed-Loop Step Responses for Varying K (Optimized B&W Visibility)
Ts = 0.01;
A = 1; B = Ts; C = -1; D = 0;

t = 0:Ts:2.5;
x0 = -0.25;                     % Initial cart position: 25 cm from wall [m]
y_ref = 0.15 * ones(size(t));   % Desired distance to wall: 15 cm [m]
x_ref = -y_ref;                 % Corresponding state reference x_ref = -0.15 [m]

K_vals = [0.5, 2.0, 20, 100, 150];

% Distinct line styles, thicknesses, and markers for B&W print clarity
line_styles = {'-', '--', '-.', '-', '-'};
line_widths = [1.8, 2.2, 1.8, 1.3, 1.2]; % Extra thick for K=2.0 (selected design)
markers     = {'none', 'none', 'none', 'o', 'x'};
marker_step = [1, 1, 1, 12, 10];         % Spacing between markers

figure('Units', 'centimeters', 'Position', [5 5 16 11]);
ax_main = axes;
hold(ax_main, 'on'); grid(ax_main, 'on'); box(ax_main, 'on');

% Plot reference line (thin dotted black)
plot(ax_main, t, y_ref, 'k:', 'LineWidth', 3, 'DisplayName', 'Reference y_{ref} [m]');

for i = 1:length(K_vals)
    Ki = K_vals(i);
    pi_val = 1 - Ts * Ki;
    
    sys_cl = ss(A - B*Ki, B*Ki, C, D, Ts);
    [y_sim, ~] = lsim(sys_cl, x_ref, t, x0);
    
    lbl = sprintf('K = %.1f s^{-1} (p_c = %.2f)', Ki, pi_val);
    
    if strcmp(markers{i}, 'none')
        plot(ax_main, t, y_sim, 'Color', 'k', 'LineStyle', line_styles{i}, ...
             'LineWidth', line_widths(i), 'DisplayName', lbl);
    else
        plot(ax_main, t, y_sim, 'Color', 'k', 'LineStyle', line_styles{i}, ...
             'LineWidth', line_widths(i), 'Marker', markers{i}, ...
             'MarkerSize', 10, 'MarkerIndices', 1:marker_step(i):length(t), ...
             'DisplayName', lbl);
    end
end

xlim(ax_main, [0 2.5]); ylim(ax_main, [0.08 0.27]);
xlabel(ax_main, 'Time [s]', 'FontSize', 30);
ylabel(ax_main, 'Measured distance y[k] [m]', 'FontSize', 30);
title(ax_main, 'Closed-loop step response for varying K', 'FontSize', 30, 'FontWeight', 'bold');
legend(ax_main, 'Location', 'Southeast', 'FontSize', 20);

% --- Optional: Zoomed Inset for Fast Transients (t = 0 to 0.15 s) ---
ax_inset = axes('Position', [0.65 0.65 0.24 0.24]);
hold(ax_inset, 'on'); grid(ax_inset, 'on'); box(ax_inset, 'on');
plot(ax_inset, t, y_ref, 'k:', 'LineWidth', 1.2);

for i = 3:5 % Plot K = 20, 100 (deadbeat), and 150 (oscillatory)
    Ki = K_vals(i);
    sys_cl = ss(A - B*Ki, B*Ki, C, D, Ts);
    [y_sim, ~] = lsim(sys_cl, x_ref, t, x0);
    
    if strcmp(markers{i}, 'none')
        plot(ax_inset, t, y_sim, 'Color', 'k', 'LineStyle', line_styles{i}, 'LineWidth', line_widths(i));
    else
        plot(ax_inset, t, y_sim, 'Color', 'k', 'LineStyle', line_styles{i}, ...
             'LineWidth', line_widths(i), 'Marker', markers{i}, 'MarkerSize', 7);
    end
end
xlim(ax_inset, [0 0.15]); ylim(ax_inset, [0.09 0.26]);
title(ax_inset, 'Zoom: t \in [0, 0.15] s', 'FontSize', 20);
set(ax_inset, 'FontSize', 20);


