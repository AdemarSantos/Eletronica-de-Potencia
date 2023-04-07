% Conversor 3L operando como compensador universal

% OBS:São introduzidos harmônicos: Em 1/3 do tempo de simulação à tensão da
% rede, em 2/3 de simulação à corrente da carga;

clear;close all;clc

%---------------- Parâmetros do Circuito RLC-RL ------------------
Rg = 0.2;               % Resistência de entrada
Lg = 5E-3;             % Indutância de entrada
Rl = 15;                 % Resistência da carga
Ll = 15E-3;              % Indutância da carga

%%%%%%%%%%
Xg = Lg*(2*pi*60);
Pl = 1000;
Vlref = 220;
FPl = 0.8;
Sl  = Pl/FPl*exp(acos(FPl)*1i);   
Zl  = conj((Vlref^2)/(Sl));	
Rl  = real(Zl);                  
Xl  = imag(Zl);                  
Ll  = Xl/(2*pi*60);
Il  = Vlref/abs(Zl);

Ig2    = roots([Rg -220 Pl]);
Ig2    = min(Ig2);
faseg = atan((-Ig2*Xg)/(220 - Ig2*Rg));
Vgref = -Ig2*Xg/sin(faseg);
%%%%%%%%%%


%---------------- Parâmetros de Simulação ------------------
h = 1E-7;                % Passo de cálculo
t = 0;                   % Tempo inicial de simulação
tf = 1;

%---------------- Parâmetros de Gravação ------------------
tsave0 = 0.;             % Tempo inicial de gravação
tsave = tsave0;          % Tempo de gravação
npt = 100000;            % Dimensão do vetor de saída de dados

hsave = (tf-tsave0)/npt; % Passo de de gravação dos vetores de saída de dados

if hsave < h             % Sendo o Passo de gravação menor que o passo de cálculo (Saturação)
    hsave = h;           % Defina o passo de gravação = passo de cálculo
    npt = (tf-tsave0)/hsave; % Recalcule o a dimensão dos vetores de saída de dados
end
n = 0;                   % Inicialização da variável de posição dos vetores de saída


%---------------- Condições do Sistema ------------------
Ed = 340;                % Tensão do barramento CC

Vl_ref = 220;             % Amplitude da tensão de referência braço L
Vgm = 220;

thetal_ref = pi/4;       % Fase da tensão de referência braço L

f_ref = 60;              % Frequência de referência
w_ref = 2*pi*f_ref;      % Frequência angular de referência

Ig = 20;
Cap = 4.4E-3;            % Capacitância do barramento

ig = 0;                  % Corrente inicial no ramo g
ilf = 0;                 % Corrente inicial no ramo l

va0_int = 0;             % valor inicial da integral da tensão no polo a
vg0_int = 0;             % valor inicial da integral da tensão no polo g
vl0_int = 0;             % valor inicial da integral da tensão no polo l

%---------------- Parâmetros do controlador ------------------
Kp = 0.2;
Ki = 15;

P_error = 0;
I_error = 0;

vc_ref = Ed;
vc = Ed;

Ig0 = 3;

%---------------- Inicialização da onda triangular ------------------
ttriangle = 0;           % Tempo inicial da onda triangular
ftriangle = 10E3;        % Frequência da onda triangular de 10 kHz
htriangle = 1/ftriangle; % Período da onda triangular
vtriangle = Ed/2;         % Tensão máxima da onda triangular
dtriangle = Ed/htriangle; % Derivada da onda triangular (?V/?T)
sign = -1;               % Sinal inicial da derivada (Comportamento decrescente)

atrasoPWM = 0.5*2.*pi*60*htriangle;
while t<tf
    t = t + h;
    
    egf = Vgm*sqrt(2)*cos(w_ref*t);
    egh = Vgm*sqrt(2)*cos(3*w_ref*t);
    egt = egf;
    
%     if t > tf/3               % Introdução de harmônicos à tensão da rede
%         egt = 0.6*egf + 0.2*egh;
% %          egt = 0; % simulando uma falta da rede
%     else
%         egt = egf;
%     end
    
    % Onda triangular
    if t >= ttriangle
        ttriangle = ttriangle + htriangle;
        
        % Controlador PI da corrente ig
        vc_error = vc_ref - vc;
        P_error = Kp*vc_error;
        I_error = I_error + htriangle*Ki*vc_error;
        Ig = P_error + I_error + Ig0;
        
        ig_ref = Ig*sqrt(2)*cos(w_ref*t);
        
        ig_error = (ig_ref - ig);
        
        % Tensões de referência
        %vg_ref = Vg_ref*cos(w_ref*t);
        vg_ref = egt - Rg*ig - (Lg/htriangle)*ig_error;
        vg2_ref  = sqrt(2)*Vgref*cos(2*pi*60*t + faseg + atrasoPWM);
        vl_ref = Vl_ref*sqrt(2)*cos(w_ref*t + thetal_ref);        

        % Fator de repartição
        VS_ref = [vg_ref,vl_ref,0];
        vu_ref_max =  Ed/2 - max(VS_ref);
        vu_ref_min = -Ed/2 - min(VS_ref);
        vu_ref = (vu_ref_max+vu_ref_min)/2;
        
        % Tensões de polo de referência
        va0_ref = vu_ref;
        vg0_ref = vg_ref + vu_ref;
        vl0_ref = vl_ref + vu_ref;        
        
        % Tensões médias nos braços
        va0_med = va0_int/htriangle;
        vg0_med = vg0_int/htriangle;
        vl0_med = vl0_int/htriangle;
        
        va0_int = 0;
        vg0_int = 0;
        vl0_int = 0;        
        
        % Onda triangular
        if vtriangle <= 0
           vtriangle = -(Ed/2);
           sign = 1;
        else
            vtriangle = (Ed/2);
            sign = -1;
        end
        
    end
    
    % Progressão da portadora triangular
    vtriangle = vtriangle + sign*dtriangle*h;
    
    % Estado das chaves
    if va0_ref >= vtriangle % Estado das chaves (Braço a)
        qa = 1; % Chave fechada
    else
        qa = 0; % Chave aberta
    end
    
    if vg0_ref >= vtriangle % Estado das chaves (Braço g)
        qg = 1; % Chave fechada
    else
        qg = 0; % Chave aberta
    end
    
    if vl0_ref >= vtriangle % Estado das chaves (Braço l)
        ql = 1; % Chave fechada (IGBT em condução)
    else
        ql = 0; % Chave aberta (IGBT em corte)
    end
    
    % Tensões de polo
    va0 = (2*qa - 1)*(Ed/2);
    vg0 = (2*qg - 1)*(Ed/2);
    vl0 = (2*ql - 1)*(Ed/2);
    
    % Integração das tensões de polo
    va0_int = va0_int + va0*h;
    vg0_int = vg0_int + vg0*h;
    vl0_int = vl0_int + vl0*h;
    
    % Tensões de fase
    vg = vg0 - va0;
    vl = vl0 - va0;
    
    % Integração numérica das correntes ig e il
    ig = ig*(1 - h*Rg/Lg) + (h/Lg)*(egt - vg); % Corrente do ramo g
    ilf = ilf*(1 - h*Rl/Ll) + (h/Ll)*vl;        % Corrente do ramo l   
    
%     if t > 2*tf/3              % Introdução de harmônicos à tensão da carga     
%         ilh = 2*cos(3*w_ref*t - pi/6);
%         ilh = ilh*(1 - h*Rl/Ll) + (h/Ll)*vl;
%         ilt = ilf + ilh;
%     else
%         ilt = ilf;
%     end   
    ilt = ilf;
    ia = ig - ilf;
    
%     Controle do barramento
    ic = ig*qg - ilt*ql - ia*qa;    % Corrente fornecida pelo capacitor
    vc = vc + h*(ic/Cap);
    
    if tsave <= t
        tsave = tsave + hsave;
        n = n + 1;
        Ts(n) = t;
        
        % Tensões de polo de referência
        va0_refs(n) = va0_ref;
        vg0_refs(n) = vg0_ref;
        vl0_refs(n) = vl0_ref;
        
        % Tensões de referência 
        vg_refs(n) = vg_ref;
        vl_refs(n) = vl_ref;  
        
        % Tensões de polo
        va0s(n) = va0;
        vg0s(n) = vg0;
        vl0s(n) = vl0;
        
        % Tensões de fase
        vgs(n) = vg;
        vls(n) = vl;    
        
        % Tensões médias
        va0_meds(n) = va0_med;
        vg0_meds(n) = vg0_med;
        vl0_meds(n) = vl0_med;   
        
        % Tensão de referência do barramento
%         vc_refs(n) = vc_ref;
        
        % Tensão do barramento
%         vcs(n) = vc;       % Tensão do barramento
        
        % Corrente de referência do controlador
        ig_refs(n) = ig_ref;
        
        % Correntes de linha
        ias(n) = ia;
        igs(n) = ig;
        ilts(n) = ilt;
        
        % Portadora triangular
        vtriangles(n) = vtriangle;
        
        %%%%%%%%%%%%%%%%%
        vcs(n)=vc;
        vg2_refs(n)=vg2_ref;
        %%%%%%%%%%%%%%%%%
    
    end
end

% SIMULAÇÃO DO LADO L
%---- Saída do conversor L
figure('Name','Tensão de saída do Conversor L')
plot(Ts,vl0s-va0s,Ts,vl0_meds-va0_meds,Ts,vl_refs,'r-','LineWidth',1.5),zoom
title('Tensão de saída do conversor L','FontSize',18)
legend('vl0_{pwm}','vl_{med}','vl_{ref}')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid()

% SIMULAÇÃO DO LADO G
%---- Saída do conversor G
figure('Name','Tensão de saída do Conversor G')
plot(Ts,vg0s-va0s,Ts,vg0_meds-va0_meds,Ts,vg_refs,'r-','LineWidth',1.5),zoom
title('Tensão de saída do conversor G','FontSize',18)
legend('vg0_{pwm}','vg_{med}','vg_{ref}')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid()

%---- Correntes
figure('Name','Corrente do circuito: Lado G, A e L')
plot(Ts,igs,Ts,ilts,Ts,ias,'r-','LineWidth',1),zoom
title('Corrente do circuito: ig, il e ia')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
legend('ig','il','ia','FontSize',18)
grid()

%---- Tensões médias
figure('Name','Tensões médias vg e vl')
plot(Ts,vg0_meds-va0_meds,Ts,vl0_meds-va0_meds,'r-','LineWidth',1.5),zoom
title('Tensões médias vg e vl','FontSize',18)
legend('vg_{med}','vl_{med}')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid()

%---- Corrente e tensão: braço G
figure('Name','Corrente e tensão: braço G')
plot(Ts,vg0_meds-va0_meds,Ts,vg_refs,Ts,igs,'r-','LineWidth',1.5),zoom
title('Corrente e tensão: braço G','FontSize',18)
legend('vg_{med}','vg_{ref}','ig')
xlabel("Tempo (s)")
ylabel("Tensão (V)| Corrente(A)")
%axis([0.28 0.42 -100 100])
grid()

%---- Corrente e tensão: braço L
figure('Name','Corrente e tensão: braço L')
plot(Ts,vl0_meds-va0_meds,Ts,vl_refs,Ts,ilts,'r-','LineWidth',1.5),zoom
title('Corrente e tensão: braço L','FontSize',18)
legend('vl_{med}','vl_{ref}','il')
xlabel("Tempo (s)")
ylabel("Tensão (V)| Corrente(A)")   
%axis([0.60 0.74 -100 100])
grid()

figure(10)
plot(Ts,vcs)

figure(11)
plot(Ts,vg2_refs,Ts,vg_refs)
legend('calculado','controlado')