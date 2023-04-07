clear;close all;clc;
format short

%---------------- Parâmetros do Circuito RLC ---------------
R = 50;                  % Resistência da carga
L = 0.05;                % Indutância da carga
C = 1E-3;                % Capacitância da carga

alphaS = R/(2*L);        % Frequência neperiana RLC série
alphaP = 1/(2*R*C);      % Frequência neperiana RLC paralelo
omega = 1/sqrt(L*C);     % frequência de ressonância

alpha = alphaS;          % Frequência neperiana do circuito

s1 = -alpha + sqrt(alpha^2 + omega^2);
s2 = -alpha - sqrt(alpha^2 + omega^2);

%---------------- Parâmetros de Simulação -------------------
t = 0;                   % Tempo inicial de simulação
h = 1E-6;                % Incremento de tempo da simulação
tf = 4/60;              % Tempo final de simulação

%---------------- Parâmetros de Gravação -------------------
tsave0=0.; 				 % Tempo inicial de gravação dos vetores de dados de saída 				
tsave=tsave0;
npt=5000;                % Número de pontos dos vetores de gravação
hsave = (tf-tsave0)/npt; % Período de amostragem dos vetores de gravação
if hsave < h
    hsave = h;
    npt = (tf-tsave0)/npt;
end

n = 0;                   % Índice dos vetores de dados de saída

%---------------- Condições do sistema -------------------
V = 20;                  % Tensão de entrada
f = 60;                  % Frequência
w = 2*pi*f;              % Frequência angular
ii = 0;                  % Corrente inicial no circuito
vc = 0;                  % Tensão inicial no capacitor

%---------------------------------------------------------
while t < tf             % Início da simulação
    t = t + h;           % Incremento do tempo de simulação
    
    vv = V*cos(w*t);     % Tensão de entrada
    vc = vc + (h/C)*ii;  % Tensão do capacitor
    ii = (1-(h*R/L))*ii + (h/L)*(vv-vc); % Corrente do circuito
    
    if tsave <= t        % Início da gravação dos vetores dos dados de saída
        tsave = tsave + hsave;
        n = n + 1;
        Ts(n) = t;
        vvs(n) = vv;
        iis(n) = ii;
        vcs(n) = vc;
    end
end
%---------------------------------------------------------

% Análise no domínio fasorial
s = j*w;
XL = s*L;
XC = 1/(s*C);

Vs = V;
Vcs = Vs * (XC)/(R+XL+XC);
Is = Vs/(R+XL+XC);

tL = linspace(0,tf,npt);
vvL = abs(Vs)*cos(w*tL + angle(Vs));
vcL = abs(Vcs)*cos(w*tL + angle(Vcs));
iiL = abs(Is)*cos(w*tL + angle(Is));

% Plotagem dos dados de saída
figure(1)                 %Por integração numérica
subplot(3,1,1);plot(Ts,vvs,'r-','LineWidth',1.5)
title('Tensão de entrada por integração numérica')
xlabel("Tempo (ms)")
ylabel("Corrente (mA)")
grid()
subplot(3,1,2);plot(Ts,vcs,'r-','LineWidth',1.5)
title('Tensão no capacitor por integração numérica')
xlabel("Tempo (ms)")
ylabel("Tensão (V)")
grid()
subplot(3,1,3);plot(Ts,iis,'r-','LineWidth',1.5)
title('Corrente no circuito por integração numérica')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()

figure(2)                 %Por Laplace
subplot(3,1,1);plot(tL,vvL,'r-','LineWidth',1.5)
title('Tensão de entrada por Laplace')
xlabel("Tempo (ms)")
ylabel("Corrente (mA)")
grid()
subplot(3,1,2);plot(tL,vcL,'r-','LineWidth',1.5)
title('Tensão no capacitor por Laplace')
xlabel("Tempo (ms)")
ylabel("Tensão (V)")
grid()
subplot(3,1,3);plot(tL,iiL,'r-','LineWidth',1.5)
title('Corrente no circuito por Laplace')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()