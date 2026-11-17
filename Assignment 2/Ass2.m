%% =========================================================================
% ASSIGNMENT 2: DISCRETE VELOCITY CONTROL (Z-DOMAIN)
% Team 10: Anna Adons, Ludovic Devos
% =========================================================================
clear all; close all; clc;

Ts = 0.01;     % Sampletijd van de Arduino (10 ms)
PM = 45;       % Doel faserand (deg)
APL = 15;      % Additional Phase Lag (deg)
phi_target = -180 + PM + APL; % -120°

%% -------------------------------------------------------------------------
% 1. HARDCODED PARAMETERS UIT ASSIGNMENT 1
% -------------------------------------------------------------------------
% Motor A (geïdentificeerd in Assignment 1)
a1_A = -0.6734;
b1_A = 0.6424;
Hz_A = tf([0 b1_A], [1 a1_A 0], Ts); % H_z(z) = b1 / (z * (z + a1))

% Motor B (geïdentificeerd in Assignment 1)
a1_B = -0.7034;
b1_B = 0.5828;
Hz_B = tf([0 b1_B], [1 a1_B 0], Ts);

%% -------------------------------------------------------------------------
% 2. DISCREET REGELAAR ONTWERP IN HET Z-DOMEIN
% -------------------------------------------------------------------------
s = tf('s');

% --- Ontwerp Motor A ---
phase_eq_A = @(w) rad2deg(angle(freqresp(Hz_A, w)));
w_c_A = fzero(@(w) phase_eq_A(w) - phi_target, 50);

Ti_A = tand(90 - APL) / w_c_A;
Ds_cont_A = (Ti_A * s + 1) / (Ti_A * s);
Dz_unscaled_A = c2d(Ds_cont_A, Ts, 'tustin'); % Discrete PI via Tustin

mag_at_wc_A = abs(squeeze(freqresp(Dz_unscaled_A * Hz_A, w_c_A)));
K_A_gain = 1 / mag_at_wc_A;
Dz_A = K_A_gain * Dz_unscaled_A;
L_z_A = Dz_A * Hz_A;

% --- Ontwerp Motor B ---
phase_eq_B = @(w) rad2deg(angle(freqresp(Hz_B, w)));
w_c_B = fzero(@(w) phase_eq_B(w) - phi_target, 50);

Ti_B = tand(90 - APL) / w_c_B;
Ds_cont_B = (Ti_B * s + 1) / (Ti_B * s);
Dz_unscaled_B = c2d(Ds_cont_B, Ts, 'tustin');

mag_at_wc_B = abs(squeeze(freqresp(Dz_unscaled_B * Hz_B, w_c_B)));
K_B_gain = 1 / mag_at_wc_B;
Dz_B = K_B_gain * Dz_unscaled_B;
L_z_B = Dz_B * Hz_B;

% Print resultaten in de command window
fprintf('=== DISCREET REGELAAR ONTWERP (Z-DOMAIN, Ts = 0.01s) ===\n');
fprintf('Motor A -> w_c = %.4f rad/s, Ti = %.4f s, K = %.4f\n', w_c_A, Ti_A, K_A_gain);
fprintf('Motor B -> w_c = %.4f rad/s, Ti = %.4f s, K = %.4f\n\n', w_c_B, Ti_B, K_B_gain);

%% -------------------------------------------------------------------------
% 3. BODE PLOTS GENEREREN & EXPORTUREN NAAR LATEX (Images map)
% -------------------------------------------------------------------------
if ~exist('Images', 'dir')
    mkdir('Images');
end

% Haal de werkelijke discrete marges en kruisfrequenties op
[Gm_A, Pm_A, Wcp_A, Wcg_A] = margin(L_z_A);
[Gm_B, Pm_B, Wcp_B, Wcg_B] = margin(L_z_B);

% --- Discrete Bode Plot Motor A ---
figure('Units', 'centimeters', 'Position', [5 5 16 12]);
margin(L_z_A);
ax = findall(gcf, 'Type', 'axes');
set(ax, 'XLim', [1 pi/Ts]); % Begrensd tot de Nyquist-frequentie (~314 rad/s)
set(ax, 'FontSize', 11);
for i = 1:length(ax)
    axes(ax(i));
    hold on;
    xline(Wcp_A, 'k--', 'LineWidth', 1.5); % Zwarte stippellijn op discrete Wcp
    hold off;
end
grid on;
h_text = findall(gcf, 'Type', 'text');
set(h_text, 'FontSize', 13);
title('Discrete Open-loop Bode Diagram - Motor A');
exportgraphics(gcf, 'Images/bode_A.png', 'Resolution', 300);

% --- Discrete Bode Plot Motor B ---
figure('Units', 'centimeters', 'Position', [5 5 16 12]);
margin(L_z_B);
ax = findall(gcf, 'Type', 'axes');
set(ax, 'XLim', [1 pi/Ts]);
set(ax, 'FontSize', 11);
for i = 1:length(ax)
    axes(ax(i));
    hold on;
    xline(Wcp_B, 'k--', 'LineWidth', 1.5);
    hold off;
end
grid on;
h_text = findall(gcf, 'Type', 'text');
set(h_text, 'FontSize', 13);
title('Discrete Open-loop Bode Diagram - Motor B');
exportgraphics(gcf, 'Images/bode_B.png', 'Resolution', 300);