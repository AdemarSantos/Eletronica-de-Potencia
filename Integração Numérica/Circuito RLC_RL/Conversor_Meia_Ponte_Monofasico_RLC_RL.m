clear;close all;clc;
format long

%---------------- Par�metros do Circuito RLC-RL ------------------
Ri = 50;                 % Resist�ncia da carga no ramo i
Li = 0.05;               % Indut�ncia da carga no ramo i
C = 10E-6;               % Capacit�ncia da carga
Rl = 5E3;                % Resist�ncia da carga no ramo l
Ll = 0.5;                % Indut�ncia da carga no ramo l

%H(s) = Vl/Vs
zs = roots([Ll Rl]);     % Zeros de H(s)
ps = roots([Li*Ll*C, Li*Rl*C+Ll*Ri*C, Li+Ri*Rl*C+Ll, Ri+Rl]); % P�los de H(s)

%---------------- Par�metros de Simula��o ------------------
h = 1E-6;                % Passo de c�lculo
t = 0;                   % Tempo inicial de simula��o
tf = 0.2;


%---------------- Par�metros de Grava��o ------------------
tsave0 = 0.;             % Tempo inicial de grava��o
tsave = tsave0;          % Tempo de grava��o
npt = 40000;             % Dimens�o do vetor de sa�da de dados
hsave = (tf-tsave0)/npt; % Passo de de grava��o dos vetores de sa�da de dados

if hsave < h             % Sendo o Passo de grava��o menor que o passo de c�lculo (Satura��o)
    hsave = h;           % Defina o passo de grava��o = passo de c�lculo
    npt = (tf-tsave0)/hsave; % Recalcule o a dimens�o dos vetores de sa�da de dados
end

n = 0;                   % Inicializa��o da vari�vel de posi��o dos vetores de sa�da

%---------------- Condi��es do Sistema ------------------
E = 14;                  % Tens�o do barramento CC
V_ref = 6.8;               % Amplitude inicial da tens�o de refer�ncia
f_ref = 60;              % Frequ�ncia de refer�ncia
w_ref = 2*pi*f_ref;      % Frequ�ncia angular de refer�ncia
vc = 0;                  % Tens�o inicial no capacitor
ii = 0;                  % Corrente inicial no ramo i
il = 0;                  % Corrente inicial no ramo l

%---------------- Inicializa��o da onda triangular ------------------
ttriangle = 0;           % Tempo inicial da onda triangular
ftriangle = 10E3;        % Frequ�ncia da onda triangular de 10 kHz
htriangle = 1/ftriangle; % Per�odo da onda triangular
vtriangle = E/2;         % Tens�o m�xima da onda triangular
dtriangle = E/htriangle; % Derivada da onda triangular (dV/dT)
sign = -1;               % Sinal inicial da derivada (Comportamento decrescente)

% ***
v10_int = 0;         %valor inicial da integral da tens�o de polo - bra�o 1
% ***

%---------------- In�cio do Looping de itera��es ------------------
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
        
        v10_med = v10_int/htriangle; % Valor m�dio da tens�o de polo - Bra�o 1
        v10_int=0;
        
        vs_ref  =  V_ref*cos(w_ref*t); % Tens�o de refer�ncia na sa�da do inversor
        v10_ref =  vs_ref; %+ vu_ref % Tens�o de polo
    end
    
    vtriangle = vtriangle + sign*dtriangle*h;
    
    if v10_ref >= vtriangle % Estado das chaves (Bra�o 1)
        q1 = 1; % Chave fechada (IGBT em condu��o)
    else
        q1 = 0; % Chave aberta (IGBT em corte)
    end
    
    % ---- In�cio da simula��o do sistema Inversor -> Filtro -> Carga ----
    v10 = (2*q1 - 1)*(E/2); %
    
    %integra��o das tens�es de p�lo (medi��o da tens�o gerada pelo conversor)
    v10_int = v10_int + v10*h;
    
    vv = v10;
    ii = (1-(h*Ri/Li))*ii + (h/Li)*(vv-vc); % Corrente do ramo i
    vc = vc + (h/C)*(ii-il);                % Tens�o do capacitor
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
figure('Name','Rela��o Entre Tens�o de Refer�ncia e Tens�o M�dia')
plot(Ts, v10_refs,'k-',Ts, vcs, 'b-','LineWidth',1),zoom
title('Tens�o de refer�ncia vs Tens�o na carga')
legend('Tens�o de refer�ncia','Tens�o na carga')
xlabel("Tempo (s)")
ylabel("Tens�o (V)")
axis([0 4/60 -10 10])
grid()

%{
figure('Name','Tens�o de Refer�ncia e Tens�o M�dia')
subplot(2,1,1);plot(Ts, vtriangles,'r-',Ts,vs_refs,'LineWidth',1),zoom
title('Entrada do PWM')
legend('Portadora Triangular','Modulante senoidal')
xlabel("Tempo (s)")
ylabel("Tens�o (V)")
axis([0.8E-3 2/60 -7.5 7.5])
grid()
subplot(2,1,2);plot(Ts, vvs,'r-', Ts, v10_meds,'LineWidth',1),zoom
title('Sa�da do conversor')
legend('Sa�da PWM', 'Sa�da m�dia')
xlabel("Tempo (s)")
ylabel("Tens�o (V)")
axis([0.8E-3 2/60 -7.5 7.5])
grid()
%}
%{

%---- Rela��o Entre Tens�o de Refer�ncia e Tens�o M�dia
figure('Name','Rela��o Entre Tens�o de Refer�ncia e Tens�o M�dia')
plot(Ts, v10_refs,'gx', Ts, vtriangles,'k', Ts, v10_meds,'r-',Ts,vvs,'bx','LineWidth',1),zoom
title('Rela��o entre tens�o de refer�ncia e tens�o m�dia')
legend('Tens�o de refer�ncia','Portadora Triangular','Tens�o m�dia', 'Tens�o de sa�da do conversor')
xlabel("Tempo (s)")
ylabel("Tens�o (V)")
axis([0.8E-3 1.8E-3 -8 13])
grid()
%---- Sa�da do inversor
figure('Name','Sa�da do inversor')
plot(Ts, vvs,'rx-','LineWidth',0.5),zoom
title('Sa�da do inversor')
xlabel("Tempo (s)")
ylabel("Tens�o (V)")
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
%---- Tens�o no capacitor
figure('Name','Tens�o no Capacitor')
plot(Ts,vcs,'r-','LineWidth',1)
title('Tens�o no capacitor')
xlabel("Tempo (s)")
ylabel("Tens�o (A)")
grid()
%---- Tens�o no polo 10 de refer�ncia 
figure('Name','Tens�o no Polo 10')
plot(Ts,v10_refs,Ts,v10_meds,'r-','LineWidth',1)
title('Tens�o de sa�da e m�dia no polo 10 de refer�ncia')
legend('Tens�o de sa�da','Tens�o m�dia')
xlabel("Tempo (s)")
ylabel("Tens�o (A)")
grid()
%---- Tens�o de refer�ncia e da carga
figure('Name','Tens�o de Refer�ncia e da Carga')
plot(Ts,vs_refs,Ts,vcs,'r-','LineWidth',1)
title('Tens�o de refer�ncia e da carga')
legend('Tens�o de refer�ncia','Tens�o da carga')
xlabel("Tempo (s)")
ylabel("Tens�o (A)")
grid()
%---- Tens�o do sinal triangular
figure('Name','Sinal triangular')
plot(Ts,vtriangles,'r-','LineWidth',2)
title('Sinal triangular (Portadora)')
xlabel("Tempo (s)")
ylabel("Tens�o (A)")
axis([0 8*htriangle -7 7])
grid()

%}
