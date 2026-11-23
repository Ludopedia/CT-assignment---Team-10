clear all; close all; clc;

Ts = 0.01;     % Sampletijd van de Arduino (10 ms = 100 Hz)
PM = 45;       % Doel faserand (deg)
APL = 15;      % Additional Phase Lag (deg)
phi_target = -180 + PM + APL; % -120°

%% 1. PARAMETERS UIT ASSIGNMENT 1
% -------------------------------------------------------------------------
% Motor A
a1_A = -0.6734;
b1_A = 0.6424;
Hz_A = tf([0 b1_A], [1 a1_A 0], Ts); % H_z(z) = b1 / (z * (z + a1))

% Motor B
a1_B = -0.7034;
b1_B = 0.5828;
Hz_B = tf([0 b1_B], [1 a1_B 0], Ts);

%% 2. DISCREET REGELAAR ONTWERP IN HET Z-DOMEIN
% -------------------------------------------------------------------------
s = tf('s');

% Ontwerp motor A 
phase_eq_A = @(w) rad2deg(angle(freqresp(Hz_A, w)));
w_c_A = fzero(@(w) phase_eq_A(w) - phi_target, 50);

Ti_A = tand(90 - APL) / w_c_A;
Ds_cont_A = (Ti_A * s + 1) / (Ti_A * s);
Dz_unscaled_A = c2d(Ds_cont_A, Ts, 'ZOH'); % Discrete PI via ZOH

mag_at_wc_A = abs(squeeze(freqresp(Dz_unscaled_A * Hz_A, w_c_A)));
K_A_gain = 1 / mag_at_wc_A;
Dz_A = K_A_gain * Dz_unscaled_A;
L_z_A = Dz_A * Hz_A;

% Ontwerp motor B 
phase_eq_B = @(w) rad2deg(angle(freqresp(Hz_B, w)));
w_c_B = fzero(@(w) phase_eq_B(w) - phi_target, 50);

Ti_B = tand(90 - APL) / w_c_B;
Ds_cont_B = (Ti_B * s + 1) / (Ti_B * s);
Dz_unscaled_B = c2d(Ds_cont_B, Ts, 'ZOH');

mag_at_wc_B = abs(squeeze(freqresp(Dz_unscaled_B * Hz_B, w_c_B)));
K_B_gain = 1 / mag_at_wc_B;
Dz_B = K_B_gain * Dz_unscaled_B;
L_z_B = Dz_B * Hz_B;

% Print resultaten in de command window
fprintf('=== DISCREET REGELAAR ONTWERP (Z-DOMAIN, Ts = 0.01s) ===\n');
fprintf('Motor A -> w_c = %.4f rad/s, Ti = %.4f s, Kp = %.4f\n', w_c_A, Ti_A, K_A_gain);
fprintf('Motor B -> w_c = %.4f rad/s, Ti = %.4f s, Kp = %.4f\n\n', w_c_B, Ti_B, K_B_gain);

% %% 3. BODE PLOTS GENEREREN (Motor A & B)
% % -------------------------------------------------------------------------
% [Gm_A, Pm_A, Wcp_A, Wcg_A] = margin(L_z_A);
% [Gm_B, Pm_B, Wcp_B, Wcg_B] = margin(L_z_B);
% 
% % Omrekenen van Gain Margin naar dB voor de titel
% Gm_dB_A = 20*log10(Gm_A);
% Gm_dB_B = 20*log10(Gm_B);
% 
% % --- Discrete bode plot motor A ---
% figure('Units', 'centimeters', 'Position', [5 5 16 12]);
% margin(L_z_A);
% 
% h_lines_A = findall(gcf, 'Type', 'line');
% set(h_lines_A, 'Color', 'k', 'LineWidth', 1.5);
% 
% raw_ax = findall(gcf, 'Type', 'axes');
% plot_ax_A = [];
% for i = 1:length(raw_ax)
%     pos = get(raw_ax(i), 'Position');
%     if length(pos) == 4
%         plot_ax_A(end+1) = raw_ax(i);
%     end
% end
% 
% % Sorteer de assen op verticale positie zonder dot-notatie
% pos_y_A = zeros(length(plot_ax_A), 1);
% for i = 1:length(plot_ax_A)
%     p = get(plot_ax_A(i), 'Position');
%     pos_y_A(i) = p(2);
% end
% [~, idx] = sort(pos_y_A, 'descend');
% plot_ax_A = plot_ax_A(idx);
% 
% for i = 1:length(plot_ax_A)
%     axes(plot_ax_A(i)); hold on;
%     xlim([1 pi/Ts]);
%     xline(Wcg_A, 'k--', 'LineWidth', 1.5);
%     hold off;
%     grid on;
%     set(plot_ax_A(i), 'FontSize', 11);
% end
% 
% ylabel(plot_ax_A(1), 'Magnitude', 'FontSize', 40, 'Color', 'k');
% ylabel(plot_ax_A(2), 'Phase [^\circ]', 'FontSize', 40, 'Color', 'k');
% xlabel(plot_ax_A(1), 'Frequency', 'FontSize', 40, 'Color', 'k');
% 
% title_str_A = sprintf('Bode diagram (Motor A) (PM = %.1f^\\circ, GM = %.1f dB, \\omega_c = %.1f rad/s)', Pm_A, Gm_dB_A, Wcg_A);
% title(plot_ax_A(1), title_str_A, 'FontSize', 40, 'FontWeight', 'bold');
% 
% 
% % --- Discrete bode plot motor B ---
% figure('Units', 'centimeters', 'Position', [5 5 16 12]);
% margin(L_z_B);
% 
% h_lines_B = findall(gcf, 'Type', 'line');
% set(h_lines_B, 'Color', 'k', 'LineWidth', 1.5);
% 
% raw_ax = findall(gcf, 'Type', 'axes');
% plot_ax_B = [];
% for i = 1:length(raw_ax)
%     pos = get(raw_ax(i), 'Position');
%     if length(pos) == 4
%         plot_ax_B(end+1) = raw_ax(i);
%     end
% end
% 
% pos_y_B = zeros(length(plot_ax_B), 1);
% for i = 1:length(plot_ax_B)
%     p = get(plot_ax_B(i), 'Position');
%     pos_y_B(i) = p(2);
% end
% [~, idx] = sort(pos_y_B, 'descend');
% plot_ax_B = plot_ax_B(idx);
% 
% for i = 1:length(plot_ax_B)
%     axes(plot_ax_B(i)); hold on;
%     xlim([1 pi/Ts]);
%     xline(Wcg_B, 'k--', 'LineWidth', 1.5);
%     hold off;
%     grid on;
%     set(plot_ax_B(i), 'FontSize', 11);
% end
% 
% ylabel(plot_ax_B(1), 'Magnitude', 'FontSize', 40, 'Color', 'k');
% ylabel(plot_ax_B(2), 'Phase [^\circ]', 'FontSize', 40, 'Color', 'k');
% xlabel(plot_ax_B(1), 'Frequency', 'FontSize', 40, 'Color', 'k');
% 
% title_str_B = sprintf('Bode diagram (Motor B) (PM = %.1f^\\circ, GM = %.1f dB, \\omega_c = %.1f rad/s)', Pm_B, Gm_dB_B, Wcg_B);
% title(plot_ax_B(1), title_str_B, 'FontSize', 40, 'FontWeight', 'bold');

%% 4. QUESTION 2(b): SIMULATIE OP BASIS VAN EXPERIMENTELE DATA
% -------------------------------------------------------------------------
data = readmatrix('data_2b_bis.csv'); 

% Lees de data in
raw_time = data(:, 1);
ref_meas = data(:, 2);
speed_A  = data(:, 3);
speed_B  = data(:, 4);
voltage_A = data(:, 5);
voltage_B = data(:, 6);


% GEFIXTE TIJDVECTOR: Maak een perfect zuivere tijdvector aan 
% die exact even lang duurt als jullie experiment, met vaste stap Ts = 0.01s
N_samples = length(raw_time);
t_meas = (0:N_samples-1)' * Ts; 

% Simulatie van het gesloten-loop systeem (Motor A)
T_closed_A = feedback(Dz_A * Hz_A, 1);
T_control_A = feedback(Dz_A, Hz_A);

[y_sim_A, ~] = lsim(T_closed_A, ref_meas, t_meas);
[u_sim_A, ~] = lsim(T_control_A, ref_meas, t_meas);
u_sat_A = max(min(u_sim_A, 12), -12);

% Simulatie van het gesloten-loop systeem (Motor B)
T_closed_B = feedback(Dz_B * Hz_B, 1);
T_control_B = feedback(Dz_B, Hz_B);

[y_sim_B, ~] = lsim(T_closed_B, ref_meas, t_meas);
[u_sim_B, ~] = lsim(T_control_B, ref_meas, t_meas);
u_sat_B = max(min(u_sim_B, 12), -12);

%% 5. FIGUREN GENEREREN VOOR HET VERSLAG (Simulatie + Echte Data)

% --- Figuur 1: Stapresponsie (Simulatie vs Echte Meting) MOTOR A ---
fig1 = figure('Units', 'centimeters', 'Position', [5 5 8 6]);
plot(t_meas, ref_meas, 'k--', 'LineWidth', 1.2); hold on;
plot(t_meas, y_sim_A, 'k-', 'LineWidth', 1.2);
plot(t_meas, speed_A, 'k:', 'LineWidth', 1.0); 
grid on;

xlim([4 7]);
ylim([-1 11]);
set(gca, 'FontSize', 9);
xlabel('Time [s]', 'FontSize', 10);
ylabel('\omega_A [rad/s]', 'FontSize', 10);
legend('Reference', 'Simulated response', 'Measured response', 'Location', 'Southeast', 'FontSize', 8, 'Box', 'off');
title('Closed-loop velocity step response (Motor A)', 'FontSize', 10);

exportgraphics(fig1, 'Images/stepresponse_A_b.png', 'Resolution', 600);


% --- Figuur 2: Stapresponsie (Simulatie vs Echte Meting) MOTOR B ---
fig2 = figure('Units', 'centimeters', 'Position', [5 5 8 6]);
plot(t_meas, ref_meas, 'k--', 'LineWidth', 1.2); hold on;
plot(t_meas, y_sim_B, 'k-', 'LineWidth', 1.2);
plot(t_meas, speed_B, 'k:', 'LineWidth', 1.0); 
grid on;

xlim([4 7]);
ylim([-1 11]);
set(gca, 'FontSize', 9);
xlabel('Time [s]', 'FontSize', 10);
ylabel('\omega_B [rad/s]', 'FontSize', 10);
legend('Reference', 'Simulated response', 'Measured response', 'Location', 'Southeast', 'FontSize', 8, 'Box', 'off');
title('Closed-loop velocity step response (Motor B)', 'FontSize', 10);

exportgraphics(fig2, 'Images/stepresponse_B_b.png', 'Resolution', 600);


% --- Figuur 3: Tracking Error MOTOR A ---
fig3 = figure('Units', 'centimeters', 'Position', [5 5 8 6]);
e_meas_A = ref_meas - speed_A; 
e_sim_A = ref_meas - y_sim_A;
plot(t_meas, e_meas_A, 'k--', 'LineWidth', 1.2); hold on;
plot(t_meas, e_sim_A, 'k-', 'LineWidth', 1.2);
grid on;

xlim([4 7]);
set(gca, 'FontSize', 9);
xlabel('Time [s]', 'FontSize', 10);
ylabel('e_A [rad/s]', 'FontSize', 10);
legend('Measured', 'Simulated', 'Location', 'Northeast', 'FontSize', 8, 'Box', 'off');
title('Closed-loop tracking error (Motor A)', 'FontSize', 10);

exportgraphics(fig3, 'Images/trackingerror_A_b.png', 'Resolution', 600);


% --- Figuur 4: Tracking Error MOTOR B ---
fig4 = figure('Units', 'centimeters', 'Position', [5 5 8 6]);
e_meas_B = ref_meas - speed_B; 
e_sim_B = ref_meas - y_sim_B;
plot(t_meas, e_meas_B, 'k--', 'LineWidth', 1.2); hold on;
plot(t_meas, e_sim_B, 'k-', 'LineWidth', 1.2);
grid on;

xlim([4 7]);
set(gca, 'FontSize', 9);
xlabel('Time [s]', 'FontSize', 10);
ylabel('e_B [rad/s]', 'FontSize', 10);
legend('Measured', 'Simulated', 'Location', 'Northeast', 'FontSize', 8, 'Box', 'off');
title('Closed-loop tracking error (Motor B)', 'FontSize', 10);

exportgraphics(fig4, 'Images/trackingerror_B_b.png', 'Resolution', 600);


% --- Figuur 5: Control Signal & Saturation MOTOR A ---
fig5 = figure('Units', 'centimeters', 'Position', [5 5 8 6]);
plot(t_meas, u_sat_A, 'k-', 'LineWidth', 1.2); hold on; 
plot(t_meas, voltage_A, 'k:', 'LineWidth', 1.0); 
yline(12, 'k-.', 'LineWidth', 1.0); 
%yline(-12, 'k-.', 'LineWidth', 1.0); 
grid on;

xlim([4 7]);
ylim([-4 14]);
set(gca, 'FontSize', 9);
xlabel('Time [s]', 'FontSize', 10);
ylabel('u_A [V]', 'FontSize', 10);
legend('Simulated control signal', 'Measured control signal', 'Saturation limits', 'Location', 'Southeast', 'FontSize', 8, 'Box', 'on');
title('Motor input voltage (Motor A)', 'FontSize', 10);

exportgraphics(fig5, 'Images/controlsignal_A_b.png', 'Resolution', 600);


% --- Figuur 6: Control Signal & Saturation MOTOR B ---
fig6 = figure('Units', 'centimeters', 'Position', [5 5 8 6]);
plot(t_meas, u_sat_B, 'k-', 'LineWidth', 1.2); hold on; 
plot(t_meas, voltage_B, 'k:', 'LineWidth', 1.0); 
yline(12, 'k-.', 'LineWidth', 1.0); 
%yline(-12, 'k-.', 'LineWidth', 1.0); 
grid on;

xlim([4 7]);
ylim([-4 14]);
set(gca, 'FontSize', 9);
xlabel('Time [s]', 'FontSize', 10);
ylabel('u_B [V]', 'FontSize', 10);
legend('Simulated control signal', 'Measured control signal', 'Saturation limits', 'Location', 'Southeast', 'FontSize', 8, 'Box', 'on');
title('Motor input voltage (Motor B)', 'FontSize', 10);

exportgraphics(fig6, 'Images/controlsignal_B_b.png', 'Resolution', 600);

%% 6. FIGUREN GENEREREN VOOR HET 2c (Simulatie + Echte Data)

% --- Figuur 1: Stapresponsie (Simulatie vs Echte Meting) MOTOR A ---
fig1 = figure('Units', 'centimeters', 'Position', [5 5 8 6]);
plot(t_meas, ref_meas, 'k--', 'LineWidth', 1.2); hold on;
plot(t_meas, y_sim_A, 'k-', 'LineWidth', 1.2);
plot(t_meas, speed_A, 'k:', 'LineWidth', 1.0); 
grid on;

xlim([4 11]);
ylim([-1 9]);
set(gca, 'FontSize', 9);
xlabel('Time [s]', 'FontSize', 10);
ylabel('\omega_A [rad/s]', 'FontSize', 10);
legend('Reference', 'Simulated response', 'Measured response', 'Location', 'Southeast', 'FontSize', 8, 'Box', 'off');
title('Closed-loop velocity step response (Motor A)', 'FontSize', 10);

exportgraphics(fig1, 'Images/stepresponse_A_c2.png', 'Resolution', 600);


% --- Figuur 2: Stapresponsie (Simulatie vs Echte Meting) MOTOR B ---
fig2 = figure('Units', 'centimeters', 'Position', [5 5 8 6]);
plot(t_meas, ref_meas, 'k--', 'LineWidth', 1.2); hold on;
plot(t_meas, y_sim_B, 'k-', 'LineWidth', 1.2);
plot(t_meas, speed_B, 'k:', 'LineWidth', 1.0); 
grid on;

xlim([4 11]);
ylim([-1 9]);
set(gca, 'FontSize', 9);
xlabel('Time [s]', 'FontSize', 10);
ylabel('\omega_B [rad/s]', 'FontSize', 10);
legend('Reference', 'Simulated response', 'Measured response', 'Location', 'Southeast', 'FontSize', 8, 'Box', 'off');
title('Closed-loop velocity step response (Motor B)', 'FontSize', 10);

exportgraphics(fig2, 'Images/stepresponse_B_c2.png', 'Resolution', 600);


% --- Figuur 3: Tracking Error MOTOR A ---
fig3 = figure('Units', 'centimeters', 'Position', [5 5 8 6]);
e_meas_A = ref_meas - speed_A; 
e_sim_A = ref_meas - y_sim_A;
plot(t_meas, e_meas_A, 'k--', 'LineWidth', 1.2); hold on;
plot(t_meas, e_sim_A, 'k-', 'LineWidth', 1.2);
grid on;

xlim([4 11]);
ylim([-4 8]);
set(gca, 'FontSize', 9);
xlabel('Time [s]', 'FontSize', 10);
ylabel('e_A [rad/s]', 'FontSize', 10);
legend('Measured', 'Simulated', 'Location', 'Northeast', 'FontSize', 8, 'Box', 'off');
title('Closed-loop tracking error (Motor A)', 'FontSize', 10);

exportgraphics(fig3, 'Images/trackingerror_A_c2.png', 'Resolution', 600);


% --- Figuur 4: Tracking Error MOTOR B ---
fig4 = figure('Units', 'centimeters', 'Position', [5 5 8 6]);
e_meas_B = ref_meas - speed_B; 
e_sim_B = ref_meas - y_sim_B;
plot(t_meas, e_meas_B, 'k--', 'LineWidth', 1.2); hold on;
plot(t_meas, e_sim_B, 'k-', 'LineWidth', 1.2);
grid on;

xlim([4 11]);
ylim([-4 8]);
set(gca, 'FontSize', 9);
xlabel('Time [s]', 'FontSize', 10);
ylabel('e_B [rad/s]', 'FontSize', 10);
legend('Measured', 'Simulated', 'Location', 'Northeast', 'FontSize', 8, 'Box', 'off');
title('Closed-loop tracking error (Motor B)', 'FontSize', 10);

exportgraphics(fig4, 'Images/trackingerror_B_c2.png', 'Resolution', 600);


% --- Figuur 5: Control Signal & Saturation MOTOR A ---
fig5 = figure('Units', 'centimeters', 'Position', [5 5 8 6]);
plot(t_meas, u_sat_A, 'k-', 'LineWidth', 1.2); hold on; 
plot(t_meas, voltage_A, 'k:', 'LineWidth', 1.0); 
yline(12, 'k-.', 'LineWidth', 1.0); 
yline(-12, 'k-.', 'LineWidth', 1.0); 
grid on;

xlim([4 11]);
ylim([-4 14]);
set(gca, 'FontSize', 9);
xlabel('Time [s]', 'FontSize', 10);
ylabel('u_A [V]', 'FontSize', 10);
legend('Simulated control signal', 'Measured control signal', 'Saturation limits', 'Location', 'Southeast', 'FontSize', 8, 'Box', 'on');
title('Motor input voltage (Motor A)', 'FontSize', 10);

exportgraphics(fig5, 'Images/controlsignal_A_c2.png', 'Resolution', 600);


% --- Figuur 6: Control Signal & Saturation MOTOR B ---
fig6 = figure('Units', 'centimeters', 'Position', [5 5 8 6]);
plot(t_meas, u_sat_B, 'k-', 'LineWidth', 1.2); hold on; 
plot(t_meas, voltage_B, 'k:', 'LineWidth', 1.0); 
yline(12, 'k-.', 'LineWidth', 1.0); 
yline(-12, 'k-.', 'LineWidth', 1.0); 
grid on;

xlim([4 11]);
ylim([-4 14]);
set(gca, 'FontSize', 9);
xlabel('Time [s]', 'FontSize', 10);
ylabel('u_B [V]', 'FontSize', 10);
legend('Simulated control signal', 'Measured control signal', 'Saturation limits', 'Location', 'Southeast', 'FontSize', 8, 'Box', 'on');
title('Motor input voltage (Motor B)', 'FontSize', 10);

exportgraphics(fig6, 'Images/controlsignal_B_c2.png', 'Resolution', 600);