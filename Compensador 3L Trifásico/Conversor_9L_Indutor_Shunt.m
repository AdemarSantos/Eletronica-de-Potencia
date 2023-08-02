% Conversor 9L trifásico - 3x 3L com neutros interconectados

clear;clc;close all;

%% Início da Simulação
tic

%% Condições do Sistema
SwSg(1:3) = 1.;                 % Sobretensão ou afundamento
Vgm(1:3) = 110*sqrt(2).*SwSg;   % Amplitude da tensão da rede
Vl_ref = 110*sqrt(2);           % Amplitude da tensão sobre a carga

Vl_ref(1:3) = Vl_ref;           % Amplitude das tensões de fase da carga
fasel_ref(1,3) = 0;             % Fases das tensões geradas pelo conversor para cada carga

f_ref = 60;                     % Frequência de referência
w_ref = 2*pi*f_ref;             % Frequência angular de referência

% Barramento 3L de cada fase
Vc3L_ref(1:3) = 180;            % Tensão de referência do barramento 3L [V]
Vc3L = Vc3L_ref;                % Tensão medida no barramento 3L         
E3L = Vc3L_ref;                 % Tensão de referência nominal do barramento 3L

% Tensão média dos barramentos
Vcmed_ref = (Vc3L_ref(1)+Vc3L_ref(2)+Vc3L_ref(3))/3; % Tenão de referência média
Vcmed = Vcmed_ref;              % Tensão média medida         
Emed = Vcmed_ref;               % Tensão de referência média nominal

Cbar = 4.4E-3;                  % Capacitância do barramento

%% Impedâncias de entrada e saída
% Assumindo cargas simétricas

% CARGA
% Dados das impedâncias - Load:
Pl = 1000;                      % Potência ativa consumida na carga [W]
FPl = 0.8; % Ind.               % Fator de potência na carga
thetal_ref = acos(FPl);             % Defasagem tensão-corrente na carga

% Potência da carga
Nl = Pl/FPl;                    % Potência aparente
Ql = Nl*sin(thetal);            % Potência reativa
Sl = Pl + 1j*Ql;                % Potência complexa

% Impedância da carga
Zl = mean((1/2).*(Vl_ref.^2)/conj(Sl));
Rl = real(Zl);                  % Resistência
Xl = imag(Zl);                  % Reatância
Ll = Xl/w_ref;                  % Indutância

% Amplitude da corrente de regime permanente - Load
Il = Vl_ref/abs(Zl);            

% Rede
% Dados das impedâncias - Grid:
Rs = 0.2;                       % Resistência
Ls = 7e-3;                      % Indutância
Xs = Ls*w_ref;                  % Reatância

% Fator de potência - Grid
FPg_ref(1:3) = 1;               % Fator de potência de referência para a rede 1
thetag_ref(1:3) = acos(FPg_ref);% Defasagem tensão-corrente de referência da rede

%% Parâmetros de Simulação
h = 1E-6;                       % Passo de cálculo
t0 = 0;                         % Tempo inicial da simulação
t = t0;                         % Tempo da simulação
tf = 1;                         % Tempo final da simulação

%% Parâmetros de Gravação
tsave0 = 0;                     % Tempo de gravação inicial
tsave = tsave0;                 % Tempo de gravação
npt = (tf-t0)*50000;            % Dimensão do vetor de saída de dados

hsave = (tf-tsave0)/npt;        % Passo de gravação dos vetores de saída de dados

if hsave < h                    % Sendo o Passo de gravação menor que o passo de cálculo (Saturação)
    hsave = h;                  % Defina o passo de gravação = passo de cálculo
    npt = (tf-tsave0)/hsave;    % Recalcule o a dimensão dos vetores de saída de dados
end

n = 0;                   % Inicialização da variável de posição dos vetores de saída

%% Inicialização das variáveis
% Grid
vg0_ref(1:3) = 0; % Tensões de referência de polo
vg0_int(1:3) = 0; % Integrais das tensões de polo
vg0_med(1:3) = 0; % Tensões médias

ig(1:3) = 0;      % Correntes

% Load
vl0_ref(1:3) = 0; % Tensões de referência de polo
vl0_int(1:3) = 0; % Integrais das tensões de polo
vl0_med(1:3) = 0; % Tensões médias 

il(1:3) = 0;      % Correntes

% Shunt
vs0_ref(1:3) = 0; % Tensões de referência de polo
vs0_int(1:3) = 0; % Integrais das tensões de polo
vs0_med(1:3) = 0; % Tensões médias

is(1:3) = 0;      % Correntes

%% Parâmetros do PI
% Ganhos proporcional e integral
Kp(1:3) = 0.2;
Ki(1:3) = 15;

% Incialização
P_error(1:3) = 0;
I_error(1:3) = 0;

% Offset do controlador
Ig(1:3) = 0;
Ig0(1:3) = 3;

%% Inicialização da onda triangular (Normalizada)
ttri = 0;           % Tempo inicial da onda triangular
ftri = 10E3;        % Frequência da onda triangular de 10 kHz
htri = 1/ftri;      % Período da onda triangular
vtri = 1/2;         % Tensão máxima da onda triangular
dtri = 1/htri;      % Derivada da onda triangular (dV/dT)
sign = -1;          % Sinal inicial da derivada (Comportamento decrescente)
atrasoPWM = 0.5*2.*pi*f_ref/ftri; % Atraso gerado no PWM

%--------------------------------------------------------------------
%% Início do Looping
while t<tf
    t = t + h;
    
    % Tensões da rede
    eg(1) = Vgm(1)*cos(w_ref*t);                   % Fase 1
    eg(2) = Vgm(2)*cos(w_ref*t + (120)*(pi/180));  % Fase 2
    eg(3) = Vgm(3)*cos(w_ref*t - (120)*(pi/180));  % Fase 3
    
    if t >= ttri
        ttri = ttri + htri;    % Atualização da onda triangular
        
        %% PI
        % Fase 1
        Vc3L_error = Vc3L_ref - Vc3L;             % Erro controle barramento 1
        P_error = Kp.*Vc3L_error;                  % Erro proporcional 1
        I_error = I_error + htri.*Ki.*Vc3L_error;  % Erro integral 1
        Ig = P_error + I_error + Ig0;            % Amplitude ig1*
        
        % Referência
        % Fase 1
        ig_ref(1) = Ig(1)*cos(w_ref*t - thetag_ref(1));    % ig1*
        
        % Fase 2
        ig_ref(2) = Ig(2)*cos(w_ref*t - thetag_ref(2) + (120)*(pi/180)); % ig2*
        
        % Fase 3
        ig_ref(3) = Ig(3)*cos(w_ref*t - thetag_ref(3) - (120)*(pi/180)); % ig3*
        
        % Erro
        ig_error = (ig_ref - ig);                 % Erro ig
        
        %
        il_ref(1) = Il*cos(w_ref*t - thetal_ref);
        il_ref(2) = Il*cos(w_ref*t - thetal_ref + (120)*(pi/180));
        il_ref(3) = Il*cos(w_ref*t - thetal_ref - (120)*(pi/180));
        
        is_ref = ig_ref - il_ref;
        is_error = is_ref - is;
        
        %% Tensão de referência na rede - Controle Preditivo
        
        % ******************************************
        vg_ref = eg - Rs.*ig - (Ls/htri).*ig_error;        
        % ******************************************
        
        % Tensão de referência na carga 
        vel_ref(1) = Vl_ref(1)*cos(w_ref*t + fasel_ref(1));
        vel_ref(2) = Vl_ref(2)*cos(w_ref*t + (120)*(pi/180) + fasel_ref(2));
        vel_ref(3) = Vl_ref(3)*cos(w_ref*t - (120)*(pi/180) + fasel_ref(3)); 
        
        vl_ref = eg - vel_ref;
        
        % ******************************************
        % Zero auxiliar sg (0s -> 0g)
        vsg_max =  Vcmed_ref - max([vg_ref(1),vg_ref(2),vg_ref(3)]);
        vsg_min = -Vcmed_ref - min([vg_ref(1),vg_ref(2),vg_ref(3)]);
        vsg_ref = (vsg_max + vsg_min)/2;
        
        % Zero auxiliar sl (0s -> 0l)
        vsl_max =  Vcmed_ref - max([vl_ref(1),vl_ref(2),vl_ref(3)]);
        vsl_min = -Vcmed_ref - min([vl_ref(1),vl_ref(2),vl_ref(3)]);
        vsl_ref = (vsl_max + vsl_min)/2;
        % ******************************************
        
        % *********************************************************
        % Tensão de refência de entrada do conversor - Malha aberta
        vgs = vg_ref + vsg_ref;
        
        % Tensão de refência de saída do conversor
        vls = vl_ref + vsl_ref;
        % *********************************************************
        
        %% Fatores de Repartição
        % Fase 1
        vxmax(1) =  E3L(1)/2 - max([vgs(1),vls(1),0]);
        vxmin(1) = -E3L(1)/2 - min([vgs(1),vls(1),0]);
        vx(1) = (vxmax(1)+vxmin(1))/2;
        
        % Fase 2
        vxmax(2) =  E3L(2)/2 - max([vgs(2),vls(2),0]);
        vxmin(2) = -E3L(2)/2 - min([vgs(2),vls(2),0]);
        vx(2) = (vxmax(2)+vxmin(2))/2;
        
        % Fase 3
        vxmax(3) =  E3L(3)/2 - max([vgs(3),vls(3),0]);
        vxmin(3) = -E3L(3)/2 - min([vgs(3),vls(3),0]);
        vx(3) = (vxmax(3)+vxmin(3))/2;
        
        %% Tensões de Referência de Polo
        % ******************************************
        vg0_ref = vx;
        vs0_ref = vg0_ref - vgs; % Shunt
        vl0_ref = vg0_ref - vls; % Serie
        % ******************************************
        
        %% Tensões médias nos braços
        % Fase 1
        vs0_med = vs0_int./htri;
        vg0_med = vg0_int./htri;
        vl0_med = vl0_int./htri;
        
        vs0_int = 0.*vs0_int;
        vg0_int = 0.*vg0_int;
        vl0_int = 0.*vl0_int;
        
        %% Onda triangular (Normalizada)
        if vtri <= 0
            vtri = -(1/2);
            sign = 1;
        else
            vtri = (1/2);
            sign = -1;
        end
    end
    
    %% Progressão da Portadora Triangular
    vtri = vtri + sign*dtri*h;
    vt3L = vtri.*Vc3L;
    
    %% Estado das Chaves
    % Fase 1
    if vs0_ref(1) >= vt3L(1) % Estado das chaves (Braço s)
        qs(1) = 1; % Chave fechada
    else
        qs(1) = 0; % Chave aberta
    end
    
    if vg0_ref(1) >= vt3L(1) % Estado das chaves (Braço g)
        qg(1) = 1; % Chave fechada
    else
        qg(1) = 0; % Chave aberta
    end
    
    if vl0_ref(1) >= vt3L(1) % Estado das chaves (Braço l)
        ql(1) = 1; % Chave fechada (IGBT em condução)
    else
        ql(1) = 0; % Chave aberta (IGBT em corte)
    end
    
    % Fase 2
    if vs0_ref(2) >= vt3L(2) % Estado das chaves (Braço s)
        qs(2) = 1; % Chave fechada
    else
        qs(2) = 0; % Chave aberta
    end
    
    if vg0_ref(2) >= vt3L(2) % Estado das chaves (Braço g)
        qg(2) = 1; % Chave fechada
    else
        qg(2) = 0; % Chave aberta
    end
    
    if vl0_ref(2) >= vt3L(2) % Estado das chaves (Braço l)
        ql(2) = 1; % Chave fechada (IGBT em condução)
    else
        ql(2) = 0; % Chave aberta (IGBT em corte)
    end
    
    % Fase 3
    if vs0_ref(3) >= vt3L(3) % Estado das chaves (Braço s)
        qs(3) = 1; % Chave fechada
    else
        qs(3) = 0; % Chave aberta
    end
    
    if vg0_ref(3) >= vt3L(3) % Estado das chaves (Braço g)
        qg(3) = 1; % Chave fechada
    else
        qg(3) = 0; % Chave aberta
    end
    
    if vl0_ref(3) >= vt3L(3) % Estado das chaves (Braço l)
        ql(3) = 1; % Chave fechada (IGBT em condução)
    else
        ql(3) = 0; % Chave aberta (IGBT em corte)
    end
    
    %% Tensões de Polo
    % Fase 1
    vs0 = (2*qs - 1).*(Vc3L/2);
    vg0 = (2*qg - 1).*(Vc3L/2);
    vl0 = (2*ql - 1).*(Vc3L/2);
    
%     % Fase 2
%     vs0(2) = (2*qs(2) - 1)*(Vc3L(2)/2);
%     vg0(2) = (2*qg(2) - 1)*(Vc3L(2)/2);
%     vl0(2) = (2*ql(2) - 1)*(Vc3L(2)/2);
% 
%     % Fase 3
%     vs0(3) = (2*qs(3) - 1)*(Vc3L(3)/2);
%     vg0(3) = (2*qg(3) - 1)*(Vc3L(3)/2);
%     vl0(3) = (2*ql(3) - 1)*(Vc3L(3)/2);
    
    %% Tensões geradas
    
    % ******************************************
    vgs = vg0 - vs0 - vsg_ref;
    vls = vg0 - vl0 - (vsg_ref - vsl_ref);
    % ******************************************
    
%     vsg_ref = (vgs1 + vgs2 + vgs3)/3;
%     vsl_ref = (vls1 + vls2 + vls3)/3;
    
    %% Integração das tensões de polo
    vs0_int = vs0_int + vs0.*h;
    vg0_int = vg0_int + vg0.*h;
    vl0_int = vl0_int + vl0.*h;
    
    %% Integração numérica das correntes ig e il
    
    % ******************************************
    is = is.*(1 - h*Rs/Ls) + (h/Ls).*(eg - vgs);
    il = il.*(1 - h*Rl/Ll) + (h/Ll).*(eg - vls); % Carga RL
    ig = is + il;
    
    % ******************************************
    
    %% Barramentos
    
    Rp3L = 5e10;
    Rs3L = 0;
    ik3L = ig.*qg - il.*ql - is.*qs;
    ic3L = (Rp3L.*ik3L - Vc3L)/(Rp3L + Rs3L);   % Corrente de entrada barramento 2L
    Vc3L = Vc3L + (h/Cbar).*ic3L;                % Tensão barramento 3L
%     Vc3L = Vc3L_ref.*1.00;
    
    %% Salvamento
    if tsave <= t
        tsave = tsave + hsave;
        n = n + 1;
        
        % Tempo da simulação
        Ts(n) = t;
        
        % Rede
        eg1s(n) = eg(1);
        eg2s(n) = eg(2);
        eg3s(n) = eg(3);
        
        % Correntes
        % Fase 1
        ig1s(n) = ig(1);
        il1s(n) = il(2);
        is1s(n) = is(3);
        
        % Fase 2
        ig2s(n) = ig(2);
        il2s(n) = il(2);
        is2s(n) = is(2);
        
        % Fase 3
        ig3s(n) = ig(3);
        il3s(n) = il(3);
        is3s(n) = is(3);
        
        
        % Tensões instantâneas de polo 
        % Fase 1
        vs10s(n) = vs0(1);
        vg10s(n) = vg0(1);
        vl10s(n) = vl0(1);

        % Fase 2
        vs20s(n) = vs0(2);
        vg20s(n) = vg0(2);
        vl20s(n) = vl0(2);

        % Fase 3
        vs30s(n) = vs0(3);
        vg30s(n) = vg0(3);
        vl30s(n) = vl0(3);
        
        
        % Tensões de referência de polo 
        % Fase 1
        vs10_refs(n) = vs0_ref(1);
        vg10_refs(n) = vg0_ref(1);
        vl10_refs(n) = vl0_ref(1);
        
        % Fase 2
        vs20_refs(n) = vs0_ref(2);
        vg20_refs(n) = vg0_ref(2);
        vl20_refs(n) = vl0_ref(2);
        
        % Fase 
        vs30_refs(n) = vs0_ref(3);
        vg30_refs(n) = vg0_ref(3);
        vl30_refs(n) = vl0_ref(3);
        
        
        % Tensões médias de polo
        % Fase 1
        vs10_meds(n) = vs0_med(1);
        vg10_meds(n) = vg0_med(1);
        vl10_meds(n) = vl0_med(1);

        % Fase 2
        vs20_meds(n) = vs0_med(2);
        vg20_meds(n) = vg0_med(2);
        vl20_meds(n) = vl0_med(2);

        % Fase 3
        vs30_meds(n) = vs0_med(3);
        vg30_meds(n) = vg0_med(3);
        vl30_meds(n) = vl0_med(3);
        
        
        % Estado das chaves
        % Fase 1
        qs1s(n) = qs(1);
        qg1s(n) = qg(1);
        ql1s(n) = ql(1);

        % Fase 2
        qs2s(n) = qs(2);
        qg2s(n) = qg(2);
        ql2s(n) = ql(2);

        % Fase 3
        qs3s(n) = qs(3);
        qg3s(n) = qg(3);
        ql3s(n) = ql(3);
        
        % Portadoras
        vtris(n)  = vtri;
        vt3L1s(n) = vt3L(1);
        vt3L2s(n) = vt3L(2);
        vt3L3s(n) = vt3L(3);
        
        % Barramentos
        Vc3L1s(n) = Vc3L(1);
        Vc3L2s(n) = Vc3L(2);
        Vc3L3s(n) = Vc3L(3);
        
        Vc3L1_refs(n) = Vc3L_ref(1);
        Vc3L2_refs(n) = Vc3L_ref(2);
        Vc3L3_refs(n) = Vc3L_ref(3);
    end
end

%% Fim da Simulação
toc

%% Plots
% Rede - Tensão e corrente
figure(1)
subplot(3,1,1)
plot(Ts, eg1s, Ts, 10*ig1s),zoom
title('Tensão e corrente da rede - Fase 1')
legend('$e_{g1}$','$i_{g1}$','Fontsize',20,'interpreter','latex')

subplot(3,1,2)
plot(Ts, eg2s, Ts, 10*ig2s),zoom
title('Tensão e corrente da rede - Fase 2')
legend('$e_{g2}$','$i_{g2}$','Fontsize',20,'interpreter','latex')

subplot(3,1,3)
plot(Ts, eg3s, Ts, 10*ig3s),zoom
title('Tensão e corrente da rede - Fase 3')
legend('$e_{g3}$','$i_{g3}$','Fontsize',20,'interpreter','latex')

% Correntes
figure(2)
subplot(3,1,1) % Fase 1
plot(Ts,ig1s,Ts,il1s,Ts,is1s)
legend('ig1','il1','is1')

subplot(3,1,2) % Fase 2
plot(Ts,ig2s,Ts,il2s,Ts,is2s)
legend('ig2','il2','is2')

subplot(3,1,3) % Fase 3
plot(Ts,ig3s,Ts,il3s,Ts,is3s)
legend('ig3','il3','is3')

% Tensões de polo instantâneas e de referência
figure(3)
subplot(3,2,1) % Fase 1
plot(Ts,vg10s-vs10s,Ts,vg10_refs-vs10_refs,Ts,vg10_meds-vs10_meds),zoom
title('vgs fase1')

subplot(3,2,2)
plot(Ts,vl10s-vs10s,Ts,vl10_refs-vs10_refs,Ts,vl10_meds-vs10_meds),zoom
title('vls fase1')

subplot(3,2,3) % Fase 2
plot(Ts,vg20s-vs20s,Ts,vg20_refs-vs20_refs,Ts,vg20_meds-vs20_meds),zoom
title('vgs fase2')

subplot(3,2,4)
plot(Ts,vl20s-vs20s,Ts,vl20_refs-vs20_refs,Ts,vl20_meds-vs20_meds),zoom
title('vls fase2')

subplot(3,2,5) % Fase 3
plot(Ts,vg30s-vs30s,Ts,vg30_refs-vs30_refs,Ts,vg30_meds-vs30_meds),zoom
title('vgs fase3')

subplot(3,2,6)
plot(Ts,vl30s-vs30s,Ts,vl30_refs-vs30_refs,Ts,vl30_meds-vs30_meds),zoom
title('vls fase3')

% Tensões de polo de referência, triangular e estado da chave
figure(4)
subplot(3,3,1)
plot(Ts,vt3L1s,Ts,vs10_refs,Ts,100*qs1s),zoom
title('s fase1')

subplot(3,3,2)
plot(Ts,vt3L1s,Ts,vg10_refs,Ts,100*qg1s),zoom
title('g fase1')

subplot(3,3,3)
plot(Ts,vt3L1s,Ts,vl10_refs,Ts,100*ql1s),zoom
title('l fase1')

subplot(3,3,4)
plot(Ts,vt3L2s,Ts,vs20_refs,Ts,100*qs2s),zoom
title('s fase2')

subplot(3,3,5)
plot(Ts,vt3L2s,Ts,vg20_refs,Ts,100*qg2s),zoom
title('g fase2')

subplot(3,3,6)
plot(Ts,vt3L2s,Ts,vl20_refs,Ts,100*ql2s),zoom
title('l fasew')

subplot(3,3,7)
plot(Ts,vt3L3s,Ts,vs30_refs,Ts,100*qs3s),zoom
title('s fase3')

subplot(3,3,8)
plot(Ts,vt3L3s,Ts,vg30_refs,Ts,100*qg3s),zoom
title('g fase3')

subplot(3,3,9)
plot(Ts,vt3L3s,Ts,vl30_refs,Ts,100*ql3s),zoom
title('l fasee')

% Barramentos
figure(5)
subplot(3,1,1)
plot(Ts,Vc3L1_refs,Ts,Vc3L1s)
title('Barramento 1')
legend('Ref','Lido')

subplot(3,1,2)
plot(Ts,Vc3L2_refs,Ts,Vc3L2s)
title('Barramento 2')
legend('Ref','Lido')

subplot(3,1,3)
plot(Ts,Vc3L3_refs,Ts,Vc3L3s)
title('Barramento 3')
legend('Ref','Lido')