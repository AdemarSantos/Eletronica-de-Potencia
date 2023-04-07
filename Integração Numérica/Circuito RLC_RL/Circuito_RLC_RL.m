clear;close all;clc;
format short

%---------------- Parâmetros do Circuito RLC-RL ------------------
Ri = 50;                % Resistência da carga no ramo i
Li = 0.05;              % Indutância da carga no ramo i
C = 1E-3;               % Capacitância da carga
Rl = 50;                % Resistência da carga no ramo l
Ll = 0.05;              % Indutância da carga no ramo l

%---------------- Parâmetros de simulação ------------------
t = 0;                  % Tempo inicial de simulação 
h = 1E-6;               % Incremento de tempo da simulação
tf = 4/60;              % Tempo final de simulação

%---------------- Parâmetros de gravação ------------------
tsave0 = 0.;            % Tempo inicial de gravação dos vetores de dados de saída
tsave = tsave0;
npt = 5E3;              % Número de pontos dos vetores de gravação
hsave = (tf-tsave0)/npt;% Período de amostragem dos vetores de gravação
if hsave < h
    hsave = h;
    npt = (tf-tsave0)/hsave;
end

n = 0;                  % Índice dos vetores de dados de saída

%---------------- Condições do sistema ------------------
V = 20;                 % Tensão da alimentação de entrada
f = 60;                 % Frequência
w = 2*pi*f;             % Frequência angular
ii = 0;                 % Corrente inicial no ramo i
vc = 0;                 % Tensão inicial no capacitor
il = 0;                 % Corrente inicial no ramo l

%-----------------------------------------------------------
while t < tf            % Início da simulação
    t = t + h;          % Incremento do tempo de simulação
    
    vv = V*cos(w*t);    % Tensão de entrada
    ii = (1-(h*Ri/Li))*ii + (h/Li)*(vv-vc); % Corrente do ramo i
    vc = vc + (h/C)*(ii-il);                % Tensão do capacitor
    il = (1-(h*Rl/Ll))*il + (h/Ll)*vc;      % Corrente do ramo l
    
    if tsave <= t       % Início da gravação dos vetores dos dados de saída
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

% Análise no domínio fasorial
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


% Plotagem dos dados de saída
figure(1)                 %Por integração numérica
subplot(4,1,1);plot(Ts,vvs,'r-','LineWidth',1.5)
title('Tensão de entrada por integração numérica')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid()
subplot(4,1,2);plot(Ts,vcs,'r-','LineWidth',1.5)
title('Tensão no capacitor por integração numérica')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid()
subplot(4,1,3);plot(Ts,iis,'r-','LineWidth',1.5)
title('Corrente no ramo i por integração numérica')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()
subplot(4,1,4);plot(Ts,ils,'r-','LineWidth',1.5)
title('Corrente no ramo l por integração numérica')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()
%{%}
figure(2)                 %Por Laplace
subplot(4,1,1);plot(tL,vvL,'r-','LineWidth',1.5)
title('Tensão de entrada por Laplace')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid()
subplot(4,1,2);plot(tL,vcL,'r-','LineWidth',1.5)
title('Tensão no capacitor por Laplace')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
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