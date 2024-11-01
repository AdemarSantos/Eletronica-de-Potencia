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
npt = 50000;             % Dimensão do vetor de saída de dados
hsave = (tf-tsave0)/npt; % Passo de de gravação dos vetores de saída de dados

if hsave < h             % Sendo o Passo de gravação menor que o passo de cálculo (Saturação)
    hsave = h;           % Defina o passo de gravação = passo de cálculo
    npt = (tf-tsave0)/hsave; % Recalcule o a dimensão dos vetores de saída de dados
end

n = 0;                   % Inicialização da variável de posição dos vetores de saída

%---------------- Condições do Sistema ------------------
E = 100;                   % Tensão do barramento CC
u = 0.5;                   % Definição do fator de repartição
V_ref = 0.95*E;          % Amplitude inicial da tensão de referência
f_ref = 20;              % Frequência de referência
w_ref = 2*pi*f_ref;      % Frequência angular de referência
vc = 0;                  % Tensão inicial no capacitor
ii = 0;                  % Corrente inicial no ramo i
il = 0;                  % Corrente inicial no ramo l

%---------------- Inicialização da onda triangular ------------------
ttriangle = 0;           % Tempo inicial da onda triangular
ftriangle = 10E3;        % Frequência da onda triangular de 10 kHz
htriangle = 1/ftriangle; % Período da onda triangular
vtriangle = E/2;         % Tensão máxima da onda triangular
dtriangle = E/htriangle; % Derivada da onda triangular (?V/?T)
sign = -1;               % Sinal inicial da derivada (Comportamento decrescente)

v10_int = 0;         %valor inicial da integral da tensão de polo - braço 1
v20_int = 0;         %valor inicial da integral da tensão de polo - braço 2

%---------------- Início do Looping de iterações ------------------
while(t < tf)
    t = t + h;
    
    if t >= ttriangle
        ttriangle = ttriangle + htriangle;
        
        if vtriangle <= 0
            vtriangle = -E/2;
            sign = 1;
        else
            vtriangle = E/2;
            sign = -1;
        end
        
        v10_med = v10_int*ftriangle; % Valor médio da tensão de polo - Braço 1
        v20_med = v20_int*ftriangle; % Valor médio da tensão de polo - Braço 2
        v10_int=0;
        v20_int=0;
        
        vs_ref  =  V_ref*cos(w_ref*t);  % Tensão de referência na saída do inversor
        
        VS_ref  = [vs_ref/2,-vs_ref/2]; 
        vu_ref_max = E/2 - max(VS_ref);
        vu_ref_min = -E/2 - min(VS_ref);
        vu_ref = u*vu_ref_max + (1-u)*vu_ref_min; %Definição da tensão de liberdade das tensões nos polos de referência
        vu_ref = vu_ref_min;
        v10_ref =  vs_ref/2 + vu_ref;
        v20_ref = -vs_ref/2 + vu_ref;
    end
    
    vtriangle = vtriangle + sign*dtriangle*h;
    
    if v10_ref >= vtriangle % Estado das chaves (Braço 1)
        q1 = 1; % Chave fechada (IGBT em condução)
    else
        q1 = 0; % Chave aberta (IGBT em corte)
    end
    
    if v20_ref >= vtriangle % Estado das chaves (Braço 2)
        q2 = 1; % Chave fechada (IGBT em condução)
    else
        q2 = 0; % Chave aberta (IGBT em corte)
    end
    
    
    % ---- Início da simulação do sistema Inversor -> Filtro -> Carga ----
    v10 = (2*q1 - 1)*(E/2);
    v20 = (2*q2 - 1)*(E/2);
    
    %integração das tensões de pólo (medição da tensão gerada pelo conversor)
    v10_int = v10_int + v10*h;
    v20_int = v20_int + v20*h;
    
    vv = v10-v20;
    ii = (1-(h*Ri/Li))*ii + (h/Li)*(vv-vc); % Corrente do ramo i
    vc = vc + (h/C)*(ii-il);                % Tensão do capacitor
    il = (1-(h*Rl/Ll))*il + (h/Ll)*vc;      % Corrente do ramo l

    if tsave <= t
        tsave = tsave + hsave;
        n = n + 1;
        Ts(n) = t;                %#ok<SAGROW>
        vvs(n) = vv;              %#ok<SAGROW>
        v10s(n) = v10;            %#ok<SAGROW>
        v20s(n) = v20;            %#ok<SAGROW>
        vcs(n) = vc;              %#ok<SAGROW>
        iis(n) = ii;              %#ok<SAGROW>
        ils(n) = il;              %#ok<SAGROW>
        vs_refs(n)=vs_ref;        %#ok<SAGROW>
        v10_meds(n)=v10_med;      %#ok<SAGROW>
        v20_meds(n)=v20_med;      %#ok<SAGROW>
        v10_refs(n)=v10_ref;      %#ok<SAGROW>
        v20_refs(n)=v20_ref;      %#ok<SAGROW>
        vtriangles(n)= vtriangle; %#ok<SAGROW>
        q1s(n) = q1;
        q2s(n) = q2;
    end
end
%Fs = logspace(0,5,npt);
%semilogx(Fs,fft(v10_meds-v20_meds))


figure(1)
ax = [31E-4 33E-4 -E E];
subplot(4,1,1);plot(Ts,vtriangles,Ts,v10_refs,Ts,v20_refs,Ts,v10_refs-v20_refs,'LineWidth',2)
legend('VTri','v10*','v20*','vs*','Location','EastOutside')
title(['u = ',num2str(u)],'FontSize',18)
axis(ax)
grid()
subplot(4,1,2);plot(Ts,v10s,'LineWidth',2)
legend('v10s','Location','NorthEastOutside')
axis(ax)
grid()
subplot(4,1,3);plot(Ts,v20s,'LineWidth',2)
legend('v20s','Location','NorthEastOutside')
axis(ax)
grid()
subplot(4,1,4);plot(Ts,v10s-v20s,'LineWidth',2)
legend('<vs>','Location','NorthEastOutside')
axis(ax)
grid()

figure(2)
plot(Ts,vvs,Ts,v10_meds-v20_meds,Ts,v10_refs-v20_refs,'r-','LineWidth',1.5),zoom
title(['u = ',num2str(u)],'FontSize',18)
legend('vs_{pwm}','vs_{med}','vs_{med}')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
axis([42E-4 42E-4+2/60 -100 100])
grid()

figure(3)
plot(Ts,v10_refs,Ts,v10_meds,'r-','LineWidth',1.5),zoom
title(['v10'],'FontSize',18)
% legend('vs_{pwm}','vs_{med}','vs_{med}')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
axis([42E-4 42E-4+2/60 -100 100])
grid()

figure(4)
plot(Ts,v20_refs,Ts,v20_meds,'r-','LineWidth',1.5),zoom
title(['v20'],'FontSize',18)
% legend('vs_{pwm}','vs_{med}','vs_{med}')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
axis([42E-4 42E-4+2/60 -100 100])
grid()

figure(5)
plot(Ts,q1s,Ts,q2s,'r-','LineWidth',1.5),zoom
title(['v20'],'FontSize',18)
% legend('vs_{pwm}','vs_{med}','vs_{med}')
xlabel("Tempo (s)")
grid()

%---- Saída do inversor
%{
figure('Name','Tensão de Saída do Inversor')
plot(Ts,vvs,Ts,v10_meds-v20_meds,'r-','LineWidth',1.5),zoom
title('Tensão de saída do inversor','FontSize',18)
legend('vs_{pwm}','vs_{med}')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
axis([42E-4 42E-4+2/60 -7 7])
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
%---- Tensão no polo 20 de referência 
figure('Name','Tensão no Polo 20')
plot(Ts,v20_refs,Ts,v20_meds,'r-','LineWidth',1)
title('Tensão de saída e média no polo 20 de referência')
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
plot(Ts,vtriangles,Ts,v10_refs,Ts,v20_refs,'r-','LineWidth',2)
title('Sinal triangular (Portadora)')
xlabel("Tempo (s)")
ylabel("Tensão (A)")
axis([0 8*htriangle -3.8 3.8])
grid()
%}
