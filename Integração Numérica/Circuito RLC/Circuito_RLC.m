clear;close all;clc;
format short

%---------------- Par�metros do Circuito RLC ---------------
R = 50;                  % Resist�ncia da carga
L = 0.05;                % Indut�ncia da carga
C = 1E-3;                % Capacit�ncia da carga

alphaS = R/(2*L);        % Frequ�ncia neperiana RLC s�rie
alphaP = 1/(2*R*C);      % Frequ�ncia neperiana RLC paralelo
omega = 1/sqrt(L*C);     % frequ�ncia de resson�ncia

alpha = alphaS;          % Frequ�ncia neperiana do circuito

s1 = -alpha + sqrt(alpha^2 + omega^2);
s2 = -alpha - sqrt(alpha^2 + omega^2);

%---------------- Par�metros de Simula��o -------------------
t = 0;                   % Tempo inicial de simula��o
h = 1E-6;                % Incremento de tempo da simula��o
tf = 4/60;              % Tempo final de simula��o

%---------------- Par�metros de Grava��o -------------------
tsave0=0.; 				 % Tempo inicial de grava��o dos vetores de dados de sa�da 				
tsave=tsave0;
npt=5000;                % N�mero de pontos dos vetores de grava��o
hsave = (tf-tsave0)/npt; % Per�odo de amostragem dos vetores de grava��o
if hsave < h
    hsave = h;
    npt = (tf-tsave0)/npt;
end

n = 0;                   % �ndice dos vetores de dados de sa�da

%---------------- Condi��es do sistema -------------------
V = 20;                  % Tens�o de entrada
f = 60;                  % Frequ�ncia
w = 2*pi*f;              % Frequ�ncia angular
ii = 0;                  % Corrente inicial no circuito
vc = 0;                  % Tens�o inicial no capacitor

%---------------------------------------------------------
while t < tf             % In�cio da simula��o
    t = t + h;           % Incremento do tempo de simula��o
    
    vv = V*cos(w*t);     % Tens�o de entrada
    vc = vc + (h/C)*ii;  % Tens�o do capacitor
    ii = (1-(h*R/L))*ii + (h/L)*(vv-vc); % Corrente do circuito
    
    if tsave <= t        % In�cio da grava��o dos vetores dos dados de sa�da
        tsave = tsave + hsave;
        n = n + 1;
        Ts(n) = t;
        vvs(n) = vv;
        iis(n) = ii;
        vcs(n) = vc;
    end
end
%---------------------------------------------------------

% An�lise no dom�nio fasorial
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

% Plotagem dos dados de sa�da
figure(1)                 %Por integra��o num�rica
subplot(3,1,1);plot(Ts,vvs,'r-','LineWidth',1.5)
title('Tens�o de entrada por integra��o num�rica')
xlabel("Tempo (ms)")
ylabel("Corrente (mA)")
grid()
subplot(3,1,2);plot(Ts,vcs,'r-','LineWidth',1.5)
title('Tens�o no capacitor por integra��o num�rica')
xlabel("Tempo (ms)")
ylabel("Tens�o (V)")
grid()
subplot(3,1,3);plot(Ts,iis,'r-','LineWidth',1.5)
title('Corrente no circuito por integra��o num�rica')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()

figure(2)                 %Por Laplace
subplot(3,1,1);plot(tL,vvL,'r-','LineWidth',1.5)
title('Tens�o de entrada por Laplace')
xlabel("Tempo (ms)")
ylabel("Corrente (mA)")
grid()
subplot(3,1,2);plot(tL,vcL,'r-','LineWidth',1.5)
title('Tens�o no capacitor por Laplace')
xlabel("Tempo (ms)")
ylabel("Tens�o (V)")
grid()
subplot(3,1,3);plot(tL,iiL,'r-','LineWidth',1.5)
title('Corrente no circuito por Laplace')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()