% Parâmetros
alpha_c = 1;          % Amplitude do sinal (assumida como 1 para este exemplo)
omega_c = 2 * pi * 1e6; % Frequência de portadora (1 MHz para este exemplo)
omega_m = 2 * pi * 1e3; % Frequência do modulador (1 kHz para este exemplo)
beta = 5;             % Índice de modulação (arbitrário para este exemplo)

% Calcule as frequências laterais e as amplitudes
k_max = floor(9/2);  % Como estamos considerando uma faixa de 9*omega_m
k_values = -k_max:k_max;
frequencies = omega_c + k_values * omega_m;
amplitudes = alpha_c * besselj(k_values, beta);

% Plotagem do espectro de amplitude
figure;
stem(frequencies/(2*pi), amplitudes, 'r*-'); % Divide por 2*pi para obter em Hz
title('Espectro de amplitude do sinal FM modulado em tom');
xlabel('Frequência (Hz)');
ylabel('Amplitude');
grid on;
