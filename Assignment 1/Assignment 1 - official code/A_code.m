clear all; close all; clear global; clc;

%% 1. Loading, preprocessing and noise detection
% -------------------------------------------------------------------------
% Sla de 2 header-regels over
data = readmatrix('test 3.csv', 'NumHeaderLines', 2);

t_raw  = data(:,1); % Tijdstempels [ms]
u_raw  = abs(data(:,2)); % Input spanning [V] (abs voor -5V pulsen)
wA_raw = abs(data(:,3)); % Snelheid Motor A 
wB_raw = abs(data(:,4)); % Snelheid Motor B 

% Automatisch detecteren waar de eerste puls begint
start_idx = find(u_raw > 1, 1, 'first');

% We willen exact 10 seconden data (5 periodes van 2 sec) vanaf de start
t_start = t_raw(start_idx);
end_idx_candidate = find(t_raw >= t_start + 10000, 1, 'first'); % 10000 ms = 10s

u_cropped  = u_raw(start_idx:end_idx_candidate);
wA_cropped = wA_raw(start_idx:end_idx_candidate);
t_cropped  = t_raw(start_idx:end_idx_candidate);

% Resamplen naar een strak, uniform tijdsgrid (Ts = 0.01s = 10ms)
Ts = 0.01; % [s]
fs = 1 / Ts;

t_sec = (t_cropped - t_cropped(1)) / 1000; % omzetten naar seconden vanaf 0
t_uniform = (0 : Ts : 10)'; % Exact 10 seconden uniform grid

u_interp  = interp1(t_sec, u_cropped, t_uniform, 'linear');
wA_interp = interp1(t_sec, wA_cropped, t_uniform, 'linear');

% Exact afsnijden op 5 periodes
num_periods = 5;
points_per_period = 200; % 2 sec / 0.01s
N_exact = points_per_period * num_periods;

t       = t_uniform(1:N_exact);
u_t     = u_interp(1:N_exact);
omega_A = wA_interp(1:N_exact);

% Tijdsdomein plot
figure(1); hold on;
sgtitle('Measured data in time domain (10 seconds)');
subplot(2,1,1); plot(t, u_t, 'LineWidth', 1);
grid on; axis tight; xlabel('t [s]'); ylabel('u_t [V]');
subplot(2,1,2); plot(t, omega_A, 'LineWidth', 1);
grid on; axis tight; xlabel('t [s]'); ylabel('\omega_A [rad/s]');

%% 2. Overlap plot voor de verschillende periodes
% -------------------------------------------------------------------------
omega_matrix = reshape(omega_A, points_per_period, num_periods); 
u_matrix     = reshape(u_t, points_per_period, num_periods);

% Bereken het gemiddelde over de 5 periodes
omega_mean = mean(omega_matrix, 2);
u_mean     = mean(u_matrix, 2);

domega_matrix = omega_matrix - repmat(omega_mean, 1, num_periods);
du_matrix     = u_matrix - repmat(u_mean, 1, num_periods);
t_period = t(1:points_per_period);

figure(2); hold on;
sgtitle('Comparison across 5 periods to assess noise');
subplot(2,2,1); plot(t_period, omega_matrix, 'LineWidth', 1);
grid on; axis tight; xlabel('t [s]'); ylabel('\omega_A [rad/s]'); title('Output per period');
subplot(2,2,3); plot(t_period, u_matrix, 'LineWidth', 1);
grid on; axis tight; xlabel('t [s]'); ylabel('u [V]'); title('Input per period');
subplot(2,2,2); plot(t_period, domega_matrix, 'LineWidth', 1);
grid on; axis tight; xlabel('t [s]'); ylabel('\Delta \omega_A'); title('Output noise deviation');
subplot(2,2,4); plot(t_period, du_matrix, 'LineWidth', 1);
grid on; axis tight; xlabel('t [s]'); ylabel('\Delta u'); title('Input noise deviation');

%% 3. LLS applied to averaged output data (unfiltered model)
% -------------------------------------------------------------------------
Y_mean = omega_mean(3:end);
Phi_mean = [-omega_mean(2:end-1), u_mean(1:end-2)];

theta_unfilt = Phi_mean \ Y_mean; % [a1; b1]
a1 = theta_unfilt(1);
b1 = theta_unfilt(2);

Num_unfilt = [0, b1];
Den_unfilt = [1, a1, 0]; % noemer: z^2 + a1*z + 0
sys_unfilt = tf(Num_unfilt, Den_unfilt, Ts);

% Bereken fysische parameters (tau en K)
tau_unfilt = -Ts / log(-a1);
K_unfilt = b1 / (1 + a1);
fprintf('ONGEFILTERD (Averaged) Model:\n');
fprintf('a1 = %.4f, b1 = %.4f => tau = %.4fs, K = %.4f\n\n', a1, b1, tau_unfilt, K_unfilt);

omega_sim_unfilt = lsim(sys_unfilt, u_t, t);

figure(3); hold on;
sgtitle('Model Validation (Motor A) – Unfiltered Model', 'Fontsize', 20);
subplot(3,1,1);
plot(t, omega_A, 'k', 'LineWidth', 1); hold on;
plot(t, omega_sim_unfilt, 'k--', 'LineWidth', 1.5);
legend('Empirical', 'Estimated', 'Location', 'SouthEast', 'Fontsize', 14);
xlabel('Time [s]', 'Fontsize', 16); ylabel('\omega_A [rad/s]', 'Fontsize', 16); axis tight; grid on;

% Zoom in op 1 transiënt (bijv. rond t=1.9 tot t=2.5)
subplot(3,1,2);
plot(t, omega_A, 'k', 'LineWidth', 1); hold on;
plot(t, omega_sim_unfilt, 'k--', 'LineWidth', 1.5);
xlim([1.9 2.5]);
legend('Empirical', 'Estimated', 'Location', 'SouthEast', 'Fontsize', 14);
xlabel('Time [s]', 'Fontsize', 16); ylabel('\omega_A [rad/s]', 'Fontsize', 16); grid on; title('Zoom on transition', 'Fontsize', 16);

% Residual Errors
subplot(3,1,3);
plot(t, omega_A - omega_sim_unfilt, 'k', 'LineWidth', 1); hold on;
legend('Error', 'Fontsize', 14);
xlabel('Time [s]', 'Fontsize', 16); ylabel('Error [rad/s]', 'Fontsize', 16); axis tight; grid on;

%% 4. LLS with Low-Pass Filter , Sanathanan Koerner (filtered models)
% -------------------------------------------------------------------------
% 4a. Low-Pass Filter (Input + Output)
f_max = 1 / (2 * pi * tau_unfilt);
cutoff = 1.5 * f_max;
[B_filt, A_filt] = butter(2, cutoff / (fs/2)); 

omega_filt = filtfilt(B_filt, A_filt, omega_mean); 
u_filt     = filtfilt(B_filt, A_filt, u_mean);

Y_filt = omega_filt(3:end);
Phi_filt = [-omega_filt(2:end-1), u_filt(1:end-2)];
theta_filt = Phi_filt \ Y_filt; 
sys_filt_both = tf([0, theta_filt(2)], [1, theta_filt(1), 0], Ts);
omega_sim_both = lsim(sys_filt_both, u_t, t);

% 4b. Sanathanan-Koerner (SK) Iterative Filter Method
n_iter = 3;           % Aantal iteraties (meestal 3 is voldoende voor convergentie)
a1_sk  = a1;          % Start met de a1 uit het ongefilterde LLS-model
b1_sk  = b1;          % Start met de b1 uit het ongefilterde LLS-model

for i = 1:n_iter
    % Definieer het iteratieve filter op basis van de noemer van de vorige stap
    A_prev = [1, a1_sk]; 
    
    % Filter de gemiddelde input en output signalen (overeenkomstig de oefenzitting)
    omega_sk_filt = filter(1, A_prev, omega_mean);
    u_sk_filt     = filter(1, A_prev, u_mean);
    
    % Opbouwen van de regressiematrix met de gefilterde data
    Y_sk   = omega_sk_filt(3:end);
    Phi_sk = [-omega_sk_filt(2:end-1), u_sk_filt(1:end-2)];
    
    % Schat nieuwe parameters via LLS
    theta_sk = Phi_sk \ Y_sk;
    a1_sk    = theta_sk(1);
    b1_sk    = theta_sk(2);
    
    % Bereken tijdelijke fysische parameters voor opvolging
    tau_sk = -Ts / log(-a1_sk);
    K_sk   = b1_sk / (1 + a1_sk);
    fprintf('Iteratie %d: a1 = %.4f, b1 = %.4f => tau = %.4fs, K = %.4f\n', i, a1_sk, b1_sk, tau_sk, K_sk);
end

% Maak het uiteindelijke SK-modelsysteem en simuleer
sys_sk       = tf([0, b1_sk], [1, a1_sk, 0], Ts);
omega_sim_sk = lsim(sys_sk, u_t, t);

% RMSE Berekeningen
RMSE_unfilt = sqrt(mean((omega_A - omega_sim_unfilt).^2));
RMSE_both   = sqrt(mean((omega_A - omega_sim_both).^2));
RMSE_sk     = sqrt(mean((omega_A - omega_sim_sk).^2));

fprintf('RMSE (Ongefilterd) = %.4f rad/s\n', RMSE_unfilt);
fprintf('RMSE (Filtered)    = %.4f rad/s\n', RMSE_both);
fprintf('RMSE (SK model)    = %.4f rad/s\n\n', RMSE_sk);


%% 5. validation & comparison plots 
% -------------------------------------------------------------------------
figure('Name', 'Model Validation - Filtered & SK Comparison', 'Position', [200 200 900 800]);
sgtitle('Model Validation (Motor A) – Filtered Models', 'FontSize', 20);

% Subplot 1: Overzicht
subplot(3,1,1);
plot(t, omega_A, 'k', 'LineWidth', 1); hold on;
plot(t, omega_sim_unfilt, 'k--', 'LineWidth', 1.2);
plot(t, omega_sim_both, 'k:', 'LineWidth', 1.5);
plot(t, omega_sim_sk, 'k-.', 'LineWidth', 1.5);
legend('Empirical', 'Unfiltered model', 'Low-pass filter model', 'SK model', 'Location', 'SouthEast', 'FontSize', 12);
xlabel('Time [s]', 'FontSize', 14); ylabel('\omega_A [rad/s]', 'FontSize', 14); axis tight; grid on;

% Subplot 2: Zoom 
subplot(3,1,2);
plot(t, omega_A, 'k', 'LineWidth', 1); hold on;
plot(t, omega_sim_unfilt, 'k--', 'LineWidth', 1.2);
plot(t, omega_sim_both, 'k:', 'LineWidth', 1.5);
plot(t, omega_sim_sk, 'k-.', 'LineWidth', 1.5);
xlim([1.9 2.5]);
legend('Empirical', 'Unfiltered model', 'Low-pass filter model', 'SK model', 'Location', 'SouthEast', 'FontSize', 12);
xlabel('Time [s]', 'FontSize', 14); ylabel('\omega_A [rad/s]', 'FontSize', 14); grid on; title('Zoom on transition', 'FontSize', 14);

% Subplot 3: Fouten 
subplot(3,1,3);
plot(t, omega_A - omega_sim_unfilt, 'k--', 'LineWidth', 1); hold on;
plot(t, omega_A - omega_sim_both, 'k:', 'LineWidth', 1.3);
plot(t, omega_A - omega_sim_sk, 'k-.', 'LineWidth', 1.3);
legend('Error unfiltered', 'Error low-pass filter', 'Error SK model', 'FontSize', 12);
xlabel('Time [s]', 'FontSize', 14); ylabel('Error [rad/s]', 'FontSize', 14); axis tight; grid on;

%% 6. input filter comparison plots
% -------------------------------------------------------------------------
figure('Name', 'Input Filter Comparison', 'Position', [250 250 800, 500]);
plot(t(1:points_per_period), u_mean, 'k-', 'LineWidth', 1.5);
hold on;
plot(t(1:points_per_period), u_filt, 'k--', 'LineWidth', 1.5);
legend('Original input', 'Filtered input', 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 16);
ylabel('Input voltage [V]', 'FontSize', 16);
title('Input signal before and after filtering', 'FontSize', 18);
grid on;
set(findall(gcf, '-property', 'FontSize'), 'FontSize', 14);