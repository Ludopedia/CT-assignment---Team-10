% Define signal parameters based on your text
f = 0.5;                % Frequency in Hz
T = 1/f;                % Period in seconds (2s)
num_cycles = 5;         % Number of repetitions
t_end = num_cycles * T; % Total duration (10s)
Ts = 0.01;              % 10 ms sample time (matching MicroOS 100 Hz rate)

% Create time vector and generate the square wave
t = 0:Ts:t_end;
% square() generates a periodic square wave with values +1 and -1
voltage = 10 * square(2 * pi * f * t); 

% Create the plot
figure;
% Use black ('k-') to comply with the black and white printing rule
plot(t, voltage, 'k-', 'LineWidth', 1.5); 

% Add readable labels with physical units
xlabel('Time [s]', 'FontSize', 11);
ylabel('Motor Voltage [V]', 'FontSize', 11);

% Set axes limits to show the 10V pulse within the 12V hardware limit
ylim([-12 12]);
xlim([0 t_end]);
grid on;

% Export as a vector PDF for crisp rendering in your LaTeX report
%exportgraphics(gcf, 'excitation_signal.pdf', 'ContentType', 'vector', 'BackgroundColor','white');