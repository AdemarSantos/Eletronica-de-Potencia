clear;close all;clc;
format long

%---------------- Parâmetros do Circuito RLC-RL ------------------
Ri = 50;                 % Resistência da carga no ramo i
Li = 0.05;               % Indutância da carga no ramo i
C = 10E-6;               % Capacitância da carga
Rl = 5E3;                % Resistência da carga no ramo l
Ll = 0.5;                % Indutância da carga no ramo l

%H(s) = Vl/Vs
zs = roots([Ll Rl]);     % Zeros de H(s)
ps = roots([Li*Ll*C, Li*Rl*C+Ll*Ri*C, Li+Ri*Rl*C+Ll, Ri+Rl]); % Pólos de H(s)

%---------------- Parâmetros de Simulação ------------------
h = 1E-6;                % Passo de cálculo
t = 0;                   % Tempo inicial de simulação
tf = 0.2;


%---------------- Parâmetros de Gravação ------------------
tsave0 = 0.;             % Tempo inicial de gravação
tsave = tsave0;          % Tempo de gravação
npt = 40000;             % Dimensão do vetor de saída de dados
hsave = (tf-tsave0)/npt; % Passo de de gravação dos vetores de saída de dados

if hsave < h             % Sendo o Passo de gravação menor que o passo de cálculo (Saturação)
    hsave = h;           % Defina o passo de gravação = passo de cálculo
    npt = (tf-tsave0)/hsave; % Recalcule o a dimensão dos vetores de saída de dados
end

n = 0;                   % Inicialização da variável de posição dos vetores de saída

%---------------- Condições do Sistema ------------------
E = 14;                  % Tensão do barramento CC
V_ref = 6.8;               % Amplitude inicial da tensão de referência
f_ref = 60;              % Frequência de referência
w_ref = 2*pi*f_ref;      % Frequência angular de referência
vc = 0;                  % Tensão inicial no capacitor
ii = 0;                  % Corrente inicial no ramo i
il = 0;                  % Corrente inicial no ramo l

%---------------- Inicialização da onda triangular ------------------
ttriangle = 0;           % Tempo inicial da onda triangular
ftriangle = 10E3;        % Frequência da onda triangular de 10 kHz
htriangle = 1/ftriangle; % Período da onda triangular
vtriangle = E/2;         % Tensão máxima da onda triangular
dtriangle = E/htriangle; % Derivada da onda triangular (dV/dT)
sign = -1;               % Sinal inicial da derivada (Comportamento decrescente)

% ***
v10_int = 0;         %valor inicial da integral da tensão de polo - braço 1
% ***

%---------------- Início do Looping de iterações ------------------
while(t < tf)
    t = t + h;
    
    %{
    if t >= tf/2     %transitorio de amplitude da tensao de referencia
        V_ref=6;
    end
    %}
    
    if t >= ttriangle
        ttriangle = ttriangle + htriangle;
        
        if vtriangle <= 0
            vtriangle = -E/2;
            sign = 1;
        else
            vtriangle = E/2;
            sign = -1;
        end
        
        v10_med = v10_int/htriangle; % Valor médio da tensão de polo - Braço 1
        v10_int=0;
        
        vs_ref  =  V_ref*cos(w_ref*t); % Tensão de referência na saída do inversor
        v10_ref =  vs_ref; %+ vu_ref % Tensão de polo
    end
    
    vtriangle = vtriangle + sign*dtriangle*h;
    
    if v10_ref >= vtriangle % Estado das chaves (Braço 1)
        q1 = 1; % Chave fechada (IGBT em condução)
    else
        q1 = 0; % Chave aberta (IGBT em corte)
    end
    
    % ---- Início da simulação do sistema Inversor -> Filtro -> Carga ----
    v10 = (2*q1 - 1)*(E/2); %
    
    %integração das tensões de pólo (medição da tensão gerada pelo conversor)
    v10_int = v10_int + v10*h;
    
    vv = v10;
    ii = (1-(h*Ri/Li))*ii + (h/Li)*(vv-vc); % Corrente do ramo i
    vc = vc + (h/C)*(ii-il);                % Tensão do capacitor
    il = (1-(h*Rl/Ll))*il + (h/Ll)*vc;      % Corrente do ramo l

    if tsave <= t
        tsave = tsave + hsave;
        n = n + 1;
        Ts(n) = t;                %#ok<SAGROW>
        vvs(n) = vv;              %#ok<SAGROW>
        vcs(n) = vc;              %#ok<SAGROW>
        iis(n) = ii;              %#ok<SAGROW>
        ils(n) = il;              %#ok<SAGROW>
        vs_refs(n)=vs_ref;        %#ok<SAGROW>
        v10_meds(n)=v10_med;      %#ok<SAGROW>
        v10_refs(n)=v10_ref;      %#ok<SAGROW>
        vtriangles(n)= vtriangle; %#ok<SAGROW>
    end
end
figure('Name','Relação Entre Tensão de Referência e Tensão Média')
plot(Ts, v10_refs,'k-',Ts, vcs, 'b-','LineWidth',1),zoom
title('Tensão de referência vs Tensão na carga')
legend('Tensão de referência','Tensão na carga')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
axis([0 4/60 -10 10])
grid()

%{
figure('Name','Tensão de Referência e Tensão Média')
subplot(2,1,1);plot(Ts, vtriangles,'r-',Ts,vs_refs,'LineWidth',1),zoom
title('Entrada do PWM')
legend('Portadora Triangular','Modulante senoidal')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
axis([0.8E-3 2/60 -7.5 7.5])
grid()
subplot(2,1,2);plot(Ts, vvs,'r-', Ts, v10_meds,'LineWidth',1),zoom
title('Saída do conversor')
legend('Saída PWM', 'Saída média')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
axis([0.8E-3 2/60 -7.5 7.5])
grid()
%}
%{

%---- Relação Entre Tensão de Referência e Tensão Média
figure('Name','Relação Entre Tensão de Referência e Tensão Média')
plot(Ts, v10_refs,'gx', Ts, vtriangles,'k', Ts, v10_meds,'r-',Ts,vvs,'bx','LineWidth',1),zoom
title('Relação entre tensão de referência e tensão média')
legend('Tensão de referência','Portadora Triangular','Tensão média', 'Tensão de saída do conversor')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
axis([0.8E-3 1.8E-3 -8 13])
grid()
%---- Saída do inversor
figure('Name','Saída do inversor')
plot(Ts, vvs,'rx-','LineWidth',0.5),zoom
title('Saída do inversor')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
axis([0.0E-3 20E-3 -8 8])
grid()
%---- Corrente do circuito
figure('Name','Corrente do Ramo i')
plot(Ts,iis,'r-','LineWidth',1)
title('Corrente do ramo i')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()
%---- Corrente da carga
figure('Name','Corrente do Ramo l')
plot(Ts,ils,'r-','LineWidth',1)
title('Corrente do ramo l (Carga)')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()
%---- Tensão no capacitor
figure('Name','Tensão no Capacitor')
plot(Ts,vcs,'r-','LineWidth',1)
title('Tensão no capacitor')
xlabel("Tempo (s)")
ylabel("Tensão (A)")
grid()
%---- Tensão no polo 10 de referência 
figure('Name','Tensão no Polo 10')
plot(Ts,v10_refs,Ts,v10_meds,'r-','LineWidth',1)
title('Tensão de saída e média no polo 10 de referência')
legend('Tensão de saída','Tensão média')
xlabel("Tempo (s)")
ylabel("Tensão (A)")
grid()
%---- Tensão de referência e da carga
figure('Name','Tensão de Referência e da Carga')
plot(Ts,vs_refs,Ts,vcs,'r-','LineWidth',1)
title('Tensão de referência e da carga')
legend('Tensão de referência','Tensão da carga')
xlabel("Tempo (s)")
ylabel("Tensão (A)")
grid()
%---- Tensão do sinal triangular
figure('Name','Sinal triangular')
plot(Ts,vtriangles,'r-','LineWidth',2)
title('Sinal triangular (Portadora)')
xlabel("Tempo (s)")
ylabel("Tensão (A)")
axis([0 8*htriangle -7 7])
grid()

%}
