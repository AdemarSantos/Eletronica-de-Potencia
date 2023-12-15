% Conversor 9L trifásico - 3x 3L com neutros interconectados

clear;clc;close all;

%% Início da Simulação
tic

%% Condições do Sistema
SwSg = 1.;               % Sobretensão ou afundamento
Vgm = 110*sqrt(2)*SwSg;  % Amplitude da tensão da rede

% Amplitude das tensões de fase da rede
Vgm1 = Vgm;
Vgm2 = Vgm;
Vgm3 = Vgm;

% Amplitude da tensão gerada sobre a carga - Malha aberta
Vg1 = Vgm; 
Vg2 = Vgm;
Vg3 = Vgm;

Vl_ref = 110*sqrt(2);    % Amplitude da tensão sobre a carga

% Amplitude das tensões de fase da carga
Vl1_ref = Vl_ref; 
Vl2_ref = Vl_ref;
Vl3_ref = Vl_ref;

% fases das tensões geradas pelo conversor para cada carga
fasel1_ref = 0;
fasel2_ref = 0;
fasel3_ref = 0;

f_ref = 60;              % Frequência de referência
w_ref = 2*pi*f_ref;      % Frequência angular de referência

% Barramento 3L de cada fase
% Fase 1
Vc3L1_ref = 180;         % Tensão de referência do barramento 3L [V]
Vc3L1 = Vc3L1_ref;       % Tensão medida no barramento 3L         
E1 = Vc3L1_ref;          % Tensão de referência nominal do barramento 3L

% Fase 2
Vc3L2_ref = 180;         % Tensão de referência do barramento 3L [V]
Vc3L2 = Vc3L2_ref;       % Tensão medida no barramento 3L         
E2 = Vc3L2_ref;          % Tensão de referência nominal do barramento 3L

% Fase 3
Vc3L3_ref = 180;         % Tensão de referência do barramento 3L [V]
Vc3L3 = Vc3L3_ref;       % Tensão medida no barramento 3L         
E3 = Vc3L3_ref;          % Tensão de referência nominal do barramento 3L

% Média
Vc9L_ref = (Vc3L1_ref+Vc3L2_ref+Vc3L3_ref)/3; % Tenão de referência média
Vc3L = Vc9L_ref;         % Tensão média medida         
E = Vc9L_ref;            % Tensão de referência média nominal

Cbar = 4.4E-3;           % Capacitância do barramento

%% Impedâncias de entrada e saída
% Assumindo cargas simétricas

% CARGA
% Dados das impedâncias da carga:
Pl = 1000;               % Potência ativa consumida na carga [W]
FPl = 0.8; % Ind.        % Fator de potência na carga
thetal = acos(FPl);      % Defasagem tensão-corrente na carga

% Potência da carga
Nl = Pl/FPl;             % Potência aparente
Ql = Nl*sin(thetal);     % Potência reativa
Sl = Pl + 1j*Ql;         % Potência complexa

% Impedância da carga
Zl = (1/2)*(Vl_ref^2)/conj(Sl);
Rl = real(Zl);           % Resistência
Xl = imag(Zl);           % Reatância
Ll = Xl/w_ref;           % Indutância

% Amplitude da corrente de regime na carga
Il = Vl_ref/abs(Zl);

% Rede
% Dados das impedâncias da rede:
Rg = 0.2;                % Resistência
Lg = 7e-3;               % Indutância
Xg = Lg*w_ref;           % Reatância

% Fator de potência
FPg1_ref = 1;                  % Fator de potência de referência para a rede 1
thetag1_ref = acos(FPg1_ref);  % Defasagem tensão-corrente de referência da rede

FPg2_ref = 1;                  % Fator de potência de referência para a rede 2
thetag2_ref = acos(FPg2_ref);  % Defasagem tensão-corrente de referência da rede

FPg3_ref = 1;                  % Fator de potência de referência para a rede 3
thetag3_ref = acos(FPg3_ref);  % Defasagem tensão-corrente de referência da rede

%% Parâmetros de Simulação
h = 1E-6;                % Passo de cálculo
t = 0;                   % Tempo inicial de simulação
tf = 1;                  % Tempo final de simulação

%% Parâmetros de Gravação
tsave0 = 0;
tsave = tsave0;          % Tempo de gravação
npt = 50000;            % Dimensão do vetor de saída de dados

hsave = (tf-tsave0)/npt; % Passo de de gravação dos vetores de saída de dados
% hsave = 10^-5;

if hsave < h             % Sendo o Passo de gravação menor que o passo de cálculo (Saturação)
    hsave = h;           % Defina o passo de gravação = passo de cálculo
    npt = (tf-tsave0)/hsave; % Recalcule o a dimensão dos vetores de saída de dados
end

n = 0;                   % Inicialização da variável de posição dos vetores de saída

%% Inicialização das variáveis
% Lado da rede (grid)
vg10_ref = 0; % Tensão de referência de polo
vg20_ref = 0;
vg30_ref = 0;

vg10_int = 0; % Integral da tensão de polo
vg20_int = 0;
vg30_int = 0;

vg10_med = 0; % Tensão média
vg20_med = 0;
vg30_med = 0;

ig1 = 0;
ig2 = 0;
ig3 = 0;

% Lado da carga (load)
vl10_ref = 0;
vl20_ref = 0;
vl30_ref = 0;

vl10_int = 0;
vl20_int = 0;
vl30_int = 0;

vl10_med = 0;
vl20_med = 0;
vl30_med = 0;

il1 = 0;
il2 = 0;
il3 = 0;

% Braço compartilhado (shunt)
vs10_ref = 0;
vs20_ref = 0;
vs30_ref = 0;

vs10_int = 0;
vs20_int = 0;
vs30_int = 0;

vs10_med = 0;
vs20_med = 0;
vs30_med = 0;

is1 = 0;
is2 = 0;
is3 = 0;

%% Parâmetros do PI
Kp1 = 0.2;
Ki1 = 15;

Kp2 = 0.2;
Ki2 = 15;

Kp3 = 0.2;
Ki3 = 15;

P_error1 = 0;
I_error1 = 0;

P_error2 = 0;
I_error2 = 0;

P_error3 = 0;
I_error3 = 0;

Ig1 = 0;
Ig01 = 3;

Ig2 = 0;
Ig02 = 3;

Ig3 = 0;
Ig03 = 3;

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
    eg1 = Vgm1*cos(w_ref*t);                   % Fase 1
    eg2 = Vgm2*cos(w_ref*t + (120)*(pi/180));  % Fase 2
    eg3 = Vgm3*cos(w_ref*t - (120)*(pi/180));  % Fase 3
    
    if t >= ttri
        ttri = ttri + htri;    % Atualização da onda triangular
        
        %% PI
        % Fase 1
        Vc3L1_error = Vc3L1_ref - Vc3L1;             % Erro controle barramento 1
        P_error1 = Kp1*Vc3L1_error;                  % Erro proporcional 1
        I_error1 = I_error1 + htri*Ki1*Vc3L1_error;  % Erro integral 1
        Ig1 = P_error1 + I_error1 + Ig01;            % Amplitude ig1*

        ig1_ref = Ig1*cos(w_ref*t - thetag1_ref);    % ig1*
        ig1_error = (ig1_ref - ig1);                 % Erro ig1
        
        % Fase 2
        Vc3L2_error = Vc3L2_ref - Vc3L2;             % Erro controle barramento 2
        P_error2 = Kp2*Vc3L2_error;                  % Erro proporcional 2
        I_error2 = I_error2 + htri*Ki2*Vc3L2_error;  % Erro integral 2
        Ig2 = P_error2 + I_error2 + Ig02;            % Amplitude ig2*

        ig2_ref = Ig2*cos(w_ref*t - thetag2_ref + (120)*(pi/180)); % ig2*
        ig2_error = (ig2_ref - ig2);                 % Erro ig2
        
        % Fase 3
        Vc3L3_error = Vc3L3_ref - Vc3L3;             % Erro controle barramento 3
        P_error3 = Kp3*Vc3L3_error;                  % Erro proporcional 3
        I_error3 = I_error3 + htri*Ki3*Vc3L3_error;  % Erro integral 3
        Ig3 = P_error3 + I_error3 + Ig03;            % Amplitude ig3*

        ig3_ref = Ig3*cos(w_ref*t - thetag3_ref - (120)*(pi/180)); % ig3*
        ig3_error = (ig3_ref - ig3);                 % Erro ig3
        
        %% Tensão de referência na rede - Controle Preditivo
        vg1_ref = eg1 - Rg*ig1 - (Lg/htri)*ig1_error;
        vg2_ref = eg2 - Rg*ig2 - (Lg/htri)*ig2_error;
        vg3_ref = eg3 - Rg*ig3 - (Lg/htri)*ig3_error;        
        
        % Tensão de referência na carga 
        vl1_ref = Vl1_ref*cos(w_ref*t + fasel1_ref);
        vl2_ref = Vl2_ref*cos(w_ref*t + (120)*(pi/180) + fasel1_ref);
        vl3_ref = Vl3_ref*cos(w_ref*t - (120)*(pi/180) + fasel1_ref); 
        
        % Zero auxiliar sg (0s -> 0g)
        vsg_max =  Vc9L_ref - max([vg1_ref,vg2_ref,vg3_ref]);
        vsg_min = -Vc9L_ref - min([vg1_ref,vg2_ref,vg3_ref]);
        vsg_ref = (vsg_max + vsg_min)/2;
        
        % Zero auxiliar sl (0s -> 0l)
        vsl_max =  Vc9L_ref - max([vl1_ref,vl2_ref,vl3_ref]);
        vsl_min = -Vc9L_ref - min([vl1_ref,vl2_ref,vl3_ref]);
        vsl_ref = (vsl_max + vsl_min)/2;
        
        % Tensão de refência de entrada do conversor - Malha aberta
        vgs1 = vg1_ref + vsg_ref;
        vgs2 = vg2_ref + vsg_ref;
        vgs3 = vg3_ref + vsg_ref;
        
        % Tensão de refência de saída do conversor
        vls1 = vl1_ref + vsl_ref;
        vls2 = vl2_ref + vsl_ref;
        vls3 = vl3_ref + vsl_ref;
        
        %% Fatores de Repartição
        % Fase 1
        vx1max =  E1/2 - max([vgs1,vls1,0]);
        vx1min = -E1/2 - min([vgs1,vls1,0]);
        vx1 = (vx1max+vx1min)/2;
        
        % Fase 2
        vx2max =  E2/2 - max([vgs2,vls2,0]);
        vx2min = -E2/2 - min([vgs2,vls2,0]);
        vx2 = (vx2max+vx2min)/2;
        
        % Fase 3
        vx3max =  E3/2 - max([vgs3,vls3,0]);
        vx3min = -E3/2 - min([vgs3,vls3,0]);
        vx3 = (vx3max+vx3min)/2;
        
        %% Tensões de Referência de Polo
        % Fase 1
        vs10_ref = vx1;
        vg10_ref = vgs1 + vx1;
        vl10_ref = vls1 + vx1;
        
        % Fase 2
        vs20_ref = vx2;
        vg20_ref = vgs2 + vx2;
        vl20_ref = vls2 + vx2;
        
        % Fase 3
        vs30_ref = vx3;
        vg30_ref = vgs3 + vx3;
        vl30_ref = vls3 + vx3;
        
        %% Tensões médias nos braços
        % Fase 1
        vs10_med = vs10_int/htri;
        vg10_med = vg10_int/htri;
        vl10_med = vl10_int/htri;
        
        vs10_int = 0;
        vg10_int = 0;
        vl10_int = 0;
        
        % Fase 2
        vs20_med = vs20_int/htri;
        vg20_med = vg20_int/htri;
        vl20_med = vl20_int/htri;
        
        vs20_int = 0;
        vg20_int = 0;
        vl20_int = 0;
        
        % Fase 3
        vs30_med = vs30_int/htri;
        vg30_med = vg30_int/htri;
        vl30_med = vl30_int/htri;

        vs30_int = 0;
        vg30_int = 0;
        vl30_int = 0;
        
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
    vt3L1 = vtri*Vc3L1;
    vt3L2 = vtri*Vc3L2;
    vt3L3 = vtri*Vc3L3;
    
    %% Estado das Chaves
    % Fase 1
    if vs10_ref >= vt3L1 % Estado das chaves (Braço s)
        qs1 = 1; % Chave fechada
    else
        qs1 = 0; % Chave aberta
    end
    
    if vg10_ref >= vt3L1 % Estado das chaves (Braço g)
        qg1 = 1; % Chave fechada
    else
        qg1 = 0; % Chave aberta
    end
    
    if vl10_ref >= vt3L1 % Estado das chaves (Braço l)
        ql1 = 1; % Chave fechada (IGBT em condução)
    else
        ql1 = 0; % Chave aberta (IGBT em corte)
    end
    
    % Fase 2
    if vs20_ref >= vt3L2 % Estado das chaves (Braço s)
        qs2 = 1; % Chave fechada
    else
        qs2 = 0; % Chave aberta
    end
    
    if vg20_ref >= vt3L2 % Estado das chaves (Braço g)
        qg2 = 1; % Chave fechada
    else
        qg2 = 0; % Chave aberta
    end
    
    if vl20_ref >= vt3L2 % Estado das chaves (Braço l)
        ql2 = 1; % Chave fechada (IGBT em condução)
    else
        ql2 = 0; % Chave aberta (IGBT em corte)
    end
    
    % Fase 3
    if vs30_ref >= vt3L3 % Estado das chaves (Braço s)
        qs3 = 1; % Chave fechada
    else
        qs3 = 0; % Chave aberta
    end
    
    if vg30_ref >= vt3L3 % Estado das chaves (Braço g)
        qg3 = 1; % Chave fechada
    else
        qg3 = 0; % Chave aberta
    end
    
    if vl30_ref >= vt3L3 % Estado das chaves (Braço l)
        ql3 = 1; % Chave fechada (IGBT em condução)
    else
        ql3 = 0; % Chave aberta (IGBT em corte)
    end
    
    %% Tensões de Polo
    % Fase 1
    vs10 = (2*qs1 - 1)*(Vc3L1/2);
    vg10 = (2*qg1 - 1)*(Vc3L1/2);
    vl10 = (2*ql1 - 1)*(Vc3L1/2);
    
    % Fase 2
    vs20 = (2*qs2 - 1)*(Vc3L2/2);
    vg20 = (2*qg2 - 1)*(Vc3L2/2);
    vl20 = (2*ql2 - 1)*(Vc3L2/2);

    % Fase 3
    vs30 = (2*qs3 - 1)*(Vc3L3/2);
    vg30 = (2*qg3 - 1)*(Vc3L3/2);
    vl30 = (2*ql3 - 1)*(Vc3L3/2);
    
    %% Tensões geradas
    % Fase 1
    vgs1 = vg10 - vs10 - vsg_ref;
    vls1 = vl10 - vs10 - vsl_ref;
    
    % Fase 2
    vgs2 = vg20 - vs20 - vsg_ref;
    vls2 = vl20 - vs20 - vsl_ref;
    
    % Fase 3
    vgs3 = vg30 - vs30 - vsg_ref;
    vls3 = vl30 - vs30 - vsl_ref;
    
%     vsg_ref = (vgs1 + vgs2 + vgs3)/3;
%     vsl_ref = (vls1 + vls2 + vls3)/3;
    
    %% Integração das tensões de polo
    % Fase 1
    vs10_int = vs10_int + vs10*h;
    vg10_int = vg10_int + vg10*h;
    vl10_int = vl10_int + vl10*h;
    
    % Fase 2
    vs20_int = vs20_int + vs20*h;
    vg20_int = vg20_int + vg20*h;
    vl20_int = vl20_int + vl20*h;
    
    % Fase 3
    vs30_int = vs30_int + vs30*h;
    vg30_int = vg30_int + vg30*h;
    vl30_int = vl30_int + vl30*h;
    
    
    %% Integração numérica das correntes ig e il
    % Fase 1
    ig1 = ig1*(1 - h*Rg/Lg) + (h/Lg)*(eg1 - vgs1);
    il1 = il1*(1 - h*Rl/Ll) + (h/Ll)*(vls1); % Carga RL
    is1 = ig1 - il1;
    
    % Fase 2
    ig2 = ig2*(1 - h*Rg/Lg) + (h/Lg)*(eg2 - vgs2);
    il2 = il2*(1 - h*Rl/Ll) + (h/Ll)*(vls2);
    is2 = ig2 - il2;
    
    % Fase 3
    ig3 = ig3*(1 - h*Rg/Lg) + (h/Lg)*(eg3 - vgs3);
    il3 = il3*(1 - h*Rl/Ll) + (h/Ll)*(vls3);
    is3 = ig3 - il3;
    
    %% Barramentos
    % Conversor 1
    Rp3L1 = 5e10;
    Rs3L1 = 0;
    ik3L1 = ig1*qg1 - il1*ql1 - is1*qs1;
    ic3L1 = (Rp3L1*ik3L1 - Vc3L1)/(Rp3L1 + Rs3L1);   % Corrente de entrada barramento 2L
    Vc3L1 = Vc3L1 + (h/Cbar)*ic3L1;                % Tensão barramento 3L
%     Vc3L1 = Vc3L1_ref*1.00;

    % Conversor 2
    Rp3L2 = 5e10;
    Rs3L2 = 0;
    ik3L2 = ig2*qg2 - il2*ql2 - is2*qs2;
    ic3L2 = (Rp3L2*ik3L2 - Vc3L2)/(Rp3L2 + Rs3L2);   % Corrente de entrada barramento 2L
    Vc3L2 = Vc3L2 + (h/Cbar)*ic3L2;                % Tensão barramento 3L
%     Vc3L2 = Vc3L2_ref*1.00;

    % Conversor 3
    Rp3L3 = 5e10;
    Rs3L3 = 0;
    ik3L3 = ig3*qg3 - il3*ql3 - is3*qs3;
    ic3L3 = (Rp3L3*ik3L3 - Vc3L3)/(Rp3L3 + Rs3L3);   % Corrente de entrada barramento 2L
    Vc3L3 = Vc3L3 + (h/Cbar)*ic3L3;                % Tensão barramento 3L
%     Vc3L3 = Vc3L3_ref*1.00;
    
    %% Salvamento
    if tsave <= t
        tsave = tsave + hsave;
        n = n + 1;
        
        % Tempo da simulação
        Ts(n) = t;
        
        % Rede
        eg1s(n) = eg1;
        eg2s(n) = eg2;
        eg3s(n) = eg3;
        
        % Correntes
        % Fase 1
        ig1s(n) = ig1;
        il1s(n) = il1;
        is1s(n) = is1;
        
        % Fase 2
        ig2s(n) = ig2;
        il2s(n) = il2;
        is2s(n) = is2;
        
        % Fase 3
        ig3s(n) = ig3;
        il3s(n) = il3;
        is3s(n) = is3;
        
        
        % Tensões instantâneas de polo 
        % Fase 1
        vs10s(n) = vs10;
        vg10s(n) = vg10;
        vl10s(n) = vl10;

        % Fase 2
        vs20s(n) = vs20;
        vg20s(n) = vg20;
        vl20s(n) = vl20;

        % Fase 3
        vs30s(n) = vs30;
        vg30s(n) = vg30;
        vl30s(n) = vl30;
        
        
        % Tensões de referência de polo 
        % Fase 1
        vs10_refs(n) = vs10_ref;
        vg10_refs(n) = vg10_ref;
        vl10_refs(n) = vl10_ref;
        
        % Fase 2
        vs20_refs(n) = vs20_ref;
        vg20_refs(n) = vg20_ref;
        vl20_refs(n) = vl20_ref;
        
        % Fase 3
        vs30_refs(n) = vs30_ref;
        vg30_refs(n) = vg30_ref;
        vl30_refs(n) = vl30_ref;
        
        % Tensões de referência entre neutros
        vsg_refs(n) = vsg_ref;
        vsl_refs(n) = vsl_ref;
        
        % Tensões médias de polo
        % Fase 1
        vs10_meds(n) = vs10_med;
        vg10_meds(n) = vg10_med;
        vl10_meds(n) = vl10_med;

        % Fase 2
        vs20_meds(n) = vs20_med;
        vg20_meds(n) = vg20_med;
        vl20_meds(n) = vl20_med;

        % Fase 3
        vs30_meds(n) = vs30_med;
        vg30_meds(n) = vg30_med;
        vl30_meds(n) = vl30_med;
        
        
        % Estado das chaves
        % Fase 1
        qs1s(n) = qs1;
        qg1s(n) = qg1;
        ql1s(n) = ql1;

        % Fase 2
        qs2s(n) = qs2;
        qg2s(n) = qg2;
        ql2s(n) = ql2;

        % Fase 3
        qs3s(n) = qs3;
        qg3s(n) = qg3;
        ql3s(n) = ql3;
        
        % Portadoras
        vtris(n)  = vtri;
        vt3L1s(n) = vt3L1;
        vt3L2s(n) = vt3L2;
        vt3L3s(n) = vt3L3;
        
        % Barramentos
        Vc3L1s(n) = Vc3L1;
        Vc3L2s(n) = Vc3L2;
        Vc3L3s(n) = Vc3L3;
        
        Vc3L1_refs(n) = Vc3L1_ref;
        Vc3L2_refs(n) = Vc3L2_ref;
        Vc3L3_refs(n) = Vc3L3_ref;
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

% Tensões geradas e de referência
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
