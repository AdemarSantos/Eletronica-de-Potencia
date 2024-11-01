clear;close all;clc

%---------------- Par�metros do Circuito RLC-RL ------------------
Rg = 50;                 % Resist�ncia de entrada
Lg = 0.05;               % Indut�ncia de entrada
Rl = 5E3;                % Resist�ncia da carga
Ll = 0.5;                % Indut�ncia da carga

%H(s) = Vl/Vs
%zs = roots([Ll Rl]);     % Zeros de H(s)
%ps = roots([Li*Ll*C, Li*Rl*C+Ll*Ri*C, Li+Ri*Rl*C+Ll, Ri+Rl]); % P�los de H(s)

%---------------- Par�metros de Simula��o ------------------
h = 1E-6;                % Passo de c�lculo
t = 0;                   % Tempo inicial de simula��o
tf = 0.2;

%---------------- Par�metros de Grava��o ------------------
tsave0 = 0.;             % Tempo inicial de grava��o
tsave = tsave0;          % Tempo de grava��o
npt = 50000;             % Dimens�o do vetor de sa�da de dados

hsave = (tf-tsave0)/npt; % Passo de de grava��o dos vetores de sa�da de dados

if hsave < h             % Sendo o Passo de grava��o menor que o passo de c�lculo (Satura��o)
    hsave = h;           % Defina o passo de grava��o = passo de c�lculo
    npt = (tf-tsave0)/hsave; % Recalcule o a dimens�o dos vetores de sa�da de dados
end

n = 0;                   % Inicializa��o da vari�vel de posi��o dos vetores de sa�da

%---------------- Condi��es do Sistema ------------------
E = 7;                   % Tens�o do barramento CC
u = 0.5 ;                   % Defini��o do fator de reparti��o

Vg_ref = 0.95*E;         % Amplitude da tens�o de refer�ncia bra�o G
Vl_ref = 0.95*E;         % Amplitude da tens�o de refer�ncia bra�o L
thetal_ref = pi/4;       % Fase da tens�o de refer�ncia bra�o L

f_ref = 60;              % Frequ�ncia de refer�ncia
w_ref = 2*pi*f_ref;      % Frequ�ncia angular de refer�ncia

ig = 0;                  % Corrente inicial no ramo g
il = 0;                  % Corrente inicial no ramo l

va0_int = 0;             % valor inicial da integral da tens�o no polo a
vg0_int = 0;             % valor inicial da integral da tens�o no polo g
vl0_int = 0;             % valor inicial da integral da tens�o no polo l

%---------------- Inicializa��o da onda triangular ------------------
ttriangle = 0;           % Tempo inicial da onda triangular
ftriangle = 10E3;        % Frequ�ncia da onda triangular de 10 kHz
htriangle = 1/ftriangle; % Per�odo da onda triangular
vtriangle = E/2;         % Tens�o m�xima da onda triangular
dtriangle = E/htriangle; % Derivada da onda triangular (?V/?T)
sign = -1;               % Sinal inicial da derivada (Comportamento decrescente)

while t<tf
    t = t + h;
    
    if vtriangle <= 0
       vtriangle = -E/2;
       sign = 1;
    else
        vtriangle = E/2;
        sign = -1;
    end    
    
    % Onda triangular
    if t >= ttriangle
        ttriangle = ttriangle + htriangle;
        

        
        % Tens�es m�dias nos bra�os
        va0_med = va0_int/htriangle;
        vg0_med = vg0_int/htriangle;
        vl0_med = vl0_int/htriangle;
        
        va0_int = 0;
        vg0_int = 0;
        vl0_int = 0;
        
        % Tens�es de refer�ncia
        vg_ref = Vg_ref*cos(w_ref*t);
        vl_ref = Vl_ref*cos(w_ref*t + thetal_ref);
        
        % Fator de reparti��o
%         VS_ref = [vg_ref,vl_ref,0];
%         vu_ref_max =  max(VS_ref);
%         vu_ref_min =  min(VS_ref);
%         vu_ref = E*(u - 1/2) - u*vu_ref_max + (u - 1)*vu_ref_min;
        vu_ref = 0;
        % Tens�es de polo de refer�ncia
        va0_ref = vu_ref;
        vg0_ref = vg_ref;
        vl0_ref = vl_ref;
        
%         vg0_ref = vg_ref + vu_ref;
%         vl0_ref = vl_ref + vu_ref;
    end
    
    vtriangle = vtriangle +sign*dtriangle*h;
    
    % Estado das chaves
    if va0_ref >= vtriangle % Estado das chaves (Bra�o a)
        qa = 1; % Chave fechada
    else
        qa = 0; % Chave aberta
    end
    
    if vg0_ref >= vtriangle % Estado das chaves (Bra�o g)
        qg = 1; % Chave fechada
    else
        qg = 0; % Chave aberta
    end
    
    if vl0_ref >= vtriangle % Estado das chaves (Bra�o l)
        ql = 1; % Chave fechada (IGBT em condu��o)
    else
        ql = 0; % Chave aberta (IGBT em corte)
    end
    
    % Tens�es de polo
    va0 = (2*qa - 1)*(E/2);
    vg0 = (2*qg - 1)*(E/2);
    vl0 = (2*ql - 1)*(E/2);
    
    % Integra��o das tens�es de polo
    va0_int = va0_int + va0*h;
    vg0_int = vg0_int + vg0*h;
    vl0_int = vl0_int + vl0*h;
    
    eg = 3*cos(2*pi*60*t);                    % Tens�o de entrada
    vg = vg0 - va0;
    vl = vl0 - va0;
    
    ig = ig*(1 - h*Rg/Lg) + (h/Lg)*(eg - vg); % Corrente do ramo g
    il = il*(1 - h*Rl/Ll) + (h/Ll)*vl;        % Corrente do ramo l   
    
    if tsave <= t
        tsave = tsave + hsave;
        n = n + 1;
        Ts(n) = t;
        va0s(n) = va0;
        vg0s(n) = vg0;
        vl0s(n) = vl0;
        vgs(n) = vg;
        vls(n) = vl;
        igs(n) = ig;
        ils(n) = il;
        va0_meds(n) = va0_med;
        vg0_meds(n) = vg0_med;
        vl0_meds(n) = vl0_med;
        va0_refs(n) = va0_ref;
        vg0_refs(n) = vg0_ref;
        vl0_refs(n) = vl0_ref;
        vtriangles(n) = vtriangle;
    end
end

% COMPARA��O ENTRE REFER�NCIA E A TRIANGULAR E O ESTADO DAS CHAVES
%{%}
figure(1)
ax = [31E-4 33E-4 -E E];
sp1=subplot(5,1,[1 2]);
sp1.Position = [0.1 0.60 0.9 0.4];
plot(Ts,vtriangles,Ts,va0_refs,Ts,vg0_refs,Ts,vl0_refs,'LineWidth',2)
legend('VTri','va0*','vg0*','vl0*','Location','EastOutside')
title(['u = ',num2str(u)],'FontSize',18)
axis(ax);
grid('minor')

sp2=subplot(5,1,3);
sp2.Position = [0.1 0.44 0.895 0.10];
plot(Ts,va0s,'','LineWidth',2)
legend('va0s','Location','NorthEastOutside')
axis(ax);
grid('minor')

sp3=subplot(5,1,4);
sp3.Position = [0.1 0.24 0.895 0.10];
plot(Ts,vg0s,'LineWidth',2)
legend('vg0s','Location','NorthEastOutside')
axis(ax);
grid('minor')

sp4=subplot(5,1,5);
sp4.Position = [0.1 0.05 0.895 0.10];
plot(Ts,vl0s,'LineWidth',2)
legend('vl0s','Location','NorthEastOutside')
axis(ax)
grid('minor')



%{
figure('Name','Tens�o de Sa�da do conversor G')
plot(Ts,vgs,Ts,vg0_meds-va0_meds,'r','LineWidth',1.5),zoom
title('Tens�o de sa�da do conversor G','FontSize',18)
legend('vg_{pwm}','vg_{med}')
xlabel("Tempo (s)")
ylabel("Tens�o (V)")
axis([42E-4 42E-4+2/60 -7 7])
grid()

figure('Name','Tens�o de Sa�da do conversor L')
plot(Ts,vls,Ts,vl0_meds,'r','LineWidth',1.5),zoom
title('Tens�o de sa�da do conversor L','FontSize',18)
legend('vl_{pwm}','vl_{med}')
xlabel("Tempo (s)")
ylabel("Tens�o (V)")
axis([42E-4 42E-4+2/60 -7 7])
grid()
%}


% SIMULA��O DO LADO G
%---- Sa�da do conversor G

figure('Name','Tens�o de sa�da do Conversor G')
plot(Ts,vgs,Ts,vg0_meds-va0_meds,'r-','LineWidth',1.5),zoom
title('Tens�o de sa�da do conversor G','FontSize',18)
legend('vg_{pwm}','vg_{med}')
xlabel("Tempo (s)")
ylabel("Tens�o (V)")
axis([42E-4 42E-4+2/60 -7.1 7.1])
grid()

%---- Corrente do circuito lado G
figure('Name','Corrente do circuito: Lado G')
plot(Ts,igs,'r-','LineWidth',1)
title('Corrente do circuito: Lado G')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()

% SIMULA��O DO LADO L
%---- Sa�da do conversor L

figure('Name','Tens�o de sa�da do Conversor G')
plot(Ts,vls,Ts,vl0_meds-va0_meds,'r-','LineWidth',1.5),zoom
title('Tens�o de sa�da do conversor L','FontSize',18)
legend('vg_{pwm}','vg_{med}')
xlabel("Tempo (s)")
ylabel("Tens�o (V)")
axis([42E-4 42E-4+2/60 -7.1 7.1])
grid()

%---- Corrente do circuito lado L
figure('Name','Corrente do circuito: Lado G')
plot(Ts,ils,'r-','LineWidth',1)
title('Corrente do circuito: Lado L')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()

%{
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
%---- Tens�o no polo 20 de refer�ncia 
figure('Name','Tens�o no Polo 20')
plot(Ts,v20_refs,Ts,v20_meds,'r-','LineWidth',1)
title('Tens�o de sa�da e m�dia no polo 20 de refer�ncia')
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
plot(Ts,vtriangles,Ts,v10_refs,Ts,v20_refs,'r-','LineWidth',2)
title('Sinal triangular (Portadora)')
xlabel("Tempo (s)")
ylabel("Tens�o (A)")
axis([0 8*htriangle -3.8 3.8])
grid()
%}