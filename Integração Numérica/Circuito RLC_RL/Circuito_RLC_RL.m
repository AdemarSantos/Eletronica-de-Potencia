clear;close all;clc;
format short

%---------------- Par�metros do Circuito RLC-RL ------------------
Ri = 50;                % Resist�ncia da carga no ramo i
Li = 0.05;              % Indut�ncia da carga no ramo i
C = 1E-3;               % Capacit�ncia da carga
Rl = 50;                % Resist�ncia da carga no ramo l
Ll = 0.05;              % Indut�ncia da carga no ramo l

%---------------- Par�metros de simula��o ------------------
t = 0;                  % Tempo inicial de simula��o 
h = 1E-6;               % Incremento de tempo da simula��o
tf = 4/60;              % Tempo final de simula��o

%---------------- Par�metros de grava��o ------------------
tsave0 = 0.;            % Tempo inicial de grava��o dos vetores de dados de sa�da
tsave = tsave0;
npt = 5E3;              % N�mero de pontos dos vetores de grava��o
hsave = (tf-tsave0)/npt;% Per�odo de amostragem dos vetores de grava��o
if hsave < h
    hsave = h;
    npt = (tf-tsave0)/hsave;
end

n = 0;                  % �ndice dos vetores de dados de sa�da

%---------------- Condi��es do sistema ------------------
V = 20;                 % Tens�o da alimenta��o de entrada
f = 60;                 % Frequ�ncia
w = 2*pi*f;             % Frequ�ncia angular
ii = 0;                 % Corrente inicial no ramo i
vc = 0;                 % Tens�o inicial no capacitor
il = 0;                 % Corrente inicial no ramo l

%-----------------------------------------------------------
while t < tf            % In�cio da simula��o
    t = t + h;          % Incremento do tempo de simula��o
    
    vv = V*cos(w*t);    % Tens�o de entrada
    ii = (1-(h*Ri/Li))*ii + (h/Li)*(vv-vc); % Corrente do ramo i
    vc = vc + (h/C)*(ii-il);                % Tens�o do capacitor
    il = (1-(h*Rl/Ll))*il + (h/Ll)*vc;      % Corrente do ramo l
    
    if tsave <= t       % In�cio da grava��o dos vetores dos dados de sa�da
        tsave = tsave + hsave;
        n = n + 1;
        Ts(n)  = t; 
        vvs(n) = vv;
        vcs(n) = vc;
        iis(n) = ii;
        ils(n) = il;
    end
end
%-----------------------------------------------------------

% An�lise no dom�nio fasorial
s = j*w;
XC = 1/(s*C);
XLi = s*Li;
XLl = s*Ll;
Zi = Ri + XLi;
Zl = Rl + XLl;
XC_Zl = (XC*Zl)/(XC+Zl);

Vs = V;
Vcs = V*XC_Zl/(Zi+XC_Zl);
Iis = V/(Zi +XC_Zl);
Ils = Vcs/Zl;

tL  = linspace(0,tf,npt);
vvL = abs(Vs)*cos(w*tL + angle(Vs));
vcL = abs(Vcs)*cos(w*tL + angle(Vcs));
iiL = abs(Iis)*cos(w*tL + angle(Iis));
ilL = abs(Ils)*cos(w*tL + angle(Ils));


% Plotagem dos dados de sa�da
figure(1)                 %Por integra��o num�rica
subplot(4,1,1);plot(Ts,vvs,'r-','LineWidth',1.5)
title('Tens�o de entrada por integra��o num�rica')
xlabel("Tempo (s)")
ylabel("Tens�o (V)")
grid()
subplot(4,1,2);plot(Ts,vcs,'r-','LineWidth',1.5)
title('Tens�o no capacitor por integra��o num�rica')
xlabel("Tempo (s)")
ylabel("Tens�o (V)")
grid()
subplot(4,1,3);plot(Ts,iis,'r-','LineWidth',1.5)
title('Corrente no ramo i por integra��o num�rica')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()
subplot(4,1,4);plot(Ts,ils,'r-','LineWidth',1.5)
title('Corrente no ramo l por integra��o num�rica')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()
%{%}
figure(2)                 %Por Laplace
subplot(4,1,1);plot(tL,vvL,'r-','LineWidth',1.5)
title('Tens�o de entrada por Laplace')
xlabel("Tempo (s)")
ylabel("Tens�o (V)")
grid()
subplot(4,1,2);plot(tL,vcL,'r-','LineWidth',1.5)
title('Tens�o no capacitor por Laplace')
xlabel("Tempo (s)")
ylabel("Tens�o (V)")
grid()
subplot(4,1,3);plot(tL,iiL,'r-','LineWidth',1.5)
title('Corrente no ramo i por Laplace')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()
subplot(4,1,4);plot(tL,ilL,'r-','LineWidth',1.5)
title('Corrente no ramo l por Laplace')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()