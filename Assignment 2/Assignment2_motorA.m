clear all; close all; clc;

% Systeemparameters
K_sys = 1.9671;
tau   = 0.0253;
Td    = 0.02;     % 20 ms totale vertraging (10ms ZOH + 10ms MicroOS)
Ts    = 0.01;     % Sampletijd van de Arduino (10 ms = 100 Hz)

% 1. Continue plant met vertraging
s = tf('s');
Gs = K_sys / (tau * s + 1);
Gs.InputDelay = Td; 

% 2. DISCRETISATIE: Zet het continue model om naar het z-domein via ZOH
Hz = c2d(Gs, Ts, 'zoh');

% Ontwerpparameters
PM  = 45;
APL = 15;
phi_target = -180 + PM + APL;

% Bepaal kruisfrequentie (in het continue equivalent voor de ontwerprichting)
phase_eq = @(w) -atand(w * tau) - (w * Td * 180/pi); 
w_c = fzero(@(w) phase_eq(w) - phi_target, 50);

% Bereken integratietijd en proportionele versterking
Ti = tand(90 - APL) / w_c;
D_unscaled = (Ti * s + 1) / (Ti * s);

% Bepaal magnitude in het continue domein bij w_c om Kp te vinden
mag_at_wc = abs(squeeze(freqresp(Gs * D_unscaled, w_c)));
Kp = 1 / mag_at_wc;

% 3. REGELAAR DISCRETISEREN: Zet de continue PI-regelaar om naar het z-domein (bijv. via Tustin/Bilinear of ZOH)
Ds_cont = Kp * D_unscaled;
Dz = c2d(Ds_cont, Ts, 'tustin'); % of 'zoh'

% 4. Open-loop Bode plot in het discrete domein (z-domein)
figure;
L_z = Dz * Hz;
margin(L_z);

% Haal de *werkelijke* discrete marges en kruisfrequentie op die de prof wil zien:
[Gm_z, Pm_z, Wcp_z, Wcg_z] = margin(L_z);
fprintf('Werkelijke discrete PM : %.2f deg\n', Pm_z);
fprintf('Werkelijke discrete GM : %.2f dB\n', 20*log10(Gm_z));

% Opmaak van de figuur
ax = findall(gcf, 'Type', 'axes');
set(ax, 'XLim', [1 pi/Ts]); % Frequentieas begrenzen tot de Nyquist-frequentie (pi/Ts = ~314 rad/s)
set(ax, 'FontSize', 11);

% Voeg nette zwarte stippellijnen toe op de discrete kruisfrequentie Wcp_z
for i = 1:length(ax)
    axes(ax(i));
    hold on;
    xline(Wcp_z, 'k--', 'LineWidth', 1.5);
    hold off;
end

grid on;
h_text = findall(gcf, 'Type', 'text');
set(h_text, 'FontSize', 13);
title('Discrete Open-loop Bode Diagram (Z-domain)');
%% 
%2. TOEGEVOEGD: Gesloten-loop stapresponsie & overschot berekenen

% Maak het gesloten-loop systeem (unity feedback)
T_closed = feedback(Ds * Gs, 1);

% Bereken de stapresponsie eigenschappen (inclusief overshoot)
info = stepinfo(T_closed);

% Print de resultaten naar de Command Window
fprintf('\n--- GESLOTEN-LOOP EIGENSCHAPPEN ---\n');
fprintf('Overshoot (Overschot)       : %.2f%%\n', info.Overshoot);
fprintf('Settling Time (Inregeltijd) : %.4f s\n', info.SettlingTime);
fprintf('Rise Time                   : %.4f s\n', info.RiseTime);

% Plot de stapresponsie in het tijdsdomein om de overschot te zien
figure;
step(T_closed);
grid on;
title(sprintf('Gesloten-loop Stapresponsie (Overshoot = %.1f%%)', info.Overshoot));
xlabel('Tijd [s]');
ylabel('Snelheid [rad/s]');
set(findall(gcf, '-property', 'FontSize'), 'FontSize', 20);