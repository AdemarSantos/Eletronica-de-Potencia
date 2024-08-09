%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ademar A. Santos Jr
% Partida Suave + Inversor de Frequência
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;close all;clc


tf = 5;                   % Tempo final de simulação
t0 = 0;                   % Tempo inicial de simulação
t = t0 : 1e-6 : tf;       % Vetor tempo

tfsoftstarter = 0.5*tf;   % Tempo do soft-starter chegar ao valor final de frequência
fx = 60;                  % Valor final de frequência do soft-starter

Af = 200;                 % Amplitude final do sinal
A0 = 0;                   % Amplitude inicial do sinal
x1 = Af*cos(2*pi*fx.*t);   % Sinal 

% figure(1);                % Sinal na frequência de interesse
% plot(t,x1)
% axis([t0 tf -1.5*A 1.5*A])

k = 0;                    % Parâmetro de salvamento

for t_ = t                % Início da simulação
    if(t_<tfsoftstarter) % Evolução da frequência e da amplitude
        f = ((fx/tfsoftstarter)/2)*t_; % Incremento da frequência
        A = ((Af/tfsoftstarter))*t_;
    else
        f = fx; % Valor final da frequência
        A = Af; % Valor final da amplitude
    end
    
    k = k+1;
    x(k) = A*cos((2*pi*f)*t_); % Sinal com soft-starter
end

figure(2); % Sinal com soft-starter
plot(t,x,[tfsoftstarter tfsoftstarter],[-1.5*A 1.5*A])
axis([t0 tf -1.5*A 1.5*A]);