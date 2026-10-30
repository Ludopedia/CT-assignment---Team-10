clear all; close all; clear global; clc;

%% 1. Loading, preprocessing and noise detection
% -------------------------------------------------------------------------
% Sla de 2 header-regels over
data = readmatrix('test 1 (slip).csv', 'NumHeaderLines', 2);

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

%% 3. LLS applied to averaged output data (Ongefilterd Model) - VOORKEURSMODEL
% Eerste orde model: w[k] = -a1*w[k-1] + b1*u[k-2]
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

%% 4. LLS met Low-Pass Filter (Gefilterd Model)
% -------------------------------------------------------------------------
% Bepaal cutoff gebaseerd op tau
f_max = 1 / (2 * pi * tau_unfilt);
cutoff = 1.5 * f_max;
fprintf('Filter frequenties:\n');
fprintf('f_max = %.4f Hz, f_cutoff = %.4f Hz\n\n', f_max, cutoff);

% 2de orde Butterworth
[B_filt, A_filt] = butter(2, cutoff / (fs/2)); 

omega_filt = filtfilt(B_filt, A_filt, omega_mean); 
u_filt     = filtfilt(B_filt, A_filt, u_mean);

Y_filt = omega_filt(3:end);
Phi_filt = [-omega_filt(2:end-1), u_filt(1:end-2)];

theta_filt = Phi_filt \ Y_filt; % [a1; b1]
a1_f = theta_filt(1);
b1_f = theta_filt(2);

Num_filt = [0, b1_f];
Den_filt = [1, a1_f, 0];
sys_filt = tf(Num_filt, Den_filt, Ts);

tau_f = -Ts / log(-a1_f);
K_f = b1_f / (1 + a1_f);
fprintf('GEFILTERD Model:\n');
fprintf('a1 = %.4f, b1 = %.4f => tau = %.4fs, K = %.4f\n', a1_f, b1_f, tau_f, K_f);

omega_sim_filt = lsim(sys_filt, u_t, t);

%% 5. Vergelijking en Validatie Plots
% -------------------------------------------------------------------------
figure(3); hold on;
sgtitle('Model Validation Comparison (Motor A)');
subplot(3,1,1);
plot(t, omega_A, 'k', 'LineWidth', 1); hold on;
plot(t, omega_sim_unfilt, 'r--', 'LineWidth', 1.5);
plot(t, omega_sim_filt, 'b--', 'LineWidth', 1.5);
legend('Empirical data', 'Unfiltered Model 2(b)', 'Filtered Model 2(c)', 'Location', 'SouthEast');
xlabel('time [s]'); ylabel('\omega_A [rad/s]'); axis tight; grid on;

% Zoom in op 1 transiënt (bijv. rond t=1.9 tot t=2.5)
subplot(3,1,2);
plot(t, omega_A, 'k', 'LineWidth', 1); hold on;
plot(t, omega_sim_unfilt, 'r--', 'LineWidth', 1.5);
plot(t, omega_sim_filt, 'b--', 'LineWidth', 1.5);
xlim([1.9 2.5]);
legend('Empirical', 'Unfilt', 'Filt', 'Location', 'SouthEast');
xlabel('time [s]'); ylabel('\omega_A [rad/s]'); grid on; title('Zoom on transition');

% Residual Errors
subplot(3,1,3);
plot(t, omega_A - omega_sim_unfilt, 'r', 'LineWidth', 1); hold on;
plot(t, omega_A - omega_sim_filt, 'b', 'LineWidth', 1);
legend('Error Unfiltered', 'Error Filtered');
xlabel('time [s]'); ylabel('Error [rad/s]'); axis tight; grid on;