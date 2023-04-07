%% Ademar A. Santos Jr.
%% Conversor 2L+3L com controle da corrente da rede e da tensão dos barramentos

clear;
close all;
format long;
clc;

%% Simulação com parâmetro variado (Sel. parâmetro)
% i = 0;
% for Vgm = 100:1:500
% for FPl = 0.1:0.005:1
% for thetal_ref = 0:0.5*pi/180:110*pi/180
% for FPg = 0.1:0.005:1
% for Vel_ref = 5:1:400
%     i = i+1;

%% Condições do Sistema
SwSg = 1.00;
Vgm = 110*sqrt(2)*SwSg;  % Tensão da rede (Vp) [V]
Vgm_nom = Vgm;
Vgma_ref = 110*sqrt(2);  % Tensão de referência na entrada do 3L 
Vel_ref = 110*sqrt(2);   % Tensão de referência braço L (Vp) [V]

f_ref = 60;              % Frequência de referência da rede [Hz]
w_ref = 2*pi*f_ref;      % Frequência angular de referência da rede [rad/s]

% Barramento 2L (Conversor B)
Vc2L_ref = 190;          % Tensão de referência do barramento 2L [V]
Vc2L = Vc2L_ref;         % Tensão medida no barramento 2L
Eb = Vc2L_ref;           % Tensão de referência nominal do barramento 2L

% Barramento 3L (Conversor A) 
Vc3L_ref = 190;          % Tensão de referência do barramento 3L [V]
Vc3L = Vc3L_ref;         % Tensão medida no barramento 3L         
Ea = Vc3L_ref;           % Tensão de referência nominal do barramento 3L

% Defasagem máxima e atual entre Vg e Vl
alpha_limite = acos( (Eb^2)/(2*(Ea+Eb)*(Eb)) ); % Defasagem máxima
thetal_ref = 0*(pi/180);      % Fase [rad] da tensão de referência - 3L Braço L


%% Impedâncias de entrada e saída
Pl = 1000;               % Potência ativa consumida na carga [W]
FPl = 0.8; % Ind.        % Fator de potência na carga
thetal = acos(FPl);      % Defasagem tensão-corrente na carga

% Potência da carga
Nl = Pl/FPl;             % Potência aparente
Ql = Nl*sin(thetal);     % Potência reativa
Sl = Pl + 1j*Ql;         % Potência complexa

% Impedância da carga
Zl = (1/2)*(Vel_ref^2)/conj(Sl);
Rl = real(Zl);           % Resistência
Xl = imag(Zl);           % Reatância
Ll = Xl/w_ref;           % Indutância

% Amplitude da corrente de regime na carga
Il = Vel_ref/abs(Zl);

% Modelo da rede
Rs = 0.2;                % Resistência
Ls = 7e-3;               % Indutância
Xs = Ls*w_ref;           % Reatância
 
FPg = 1;                 % Fator de potência da rede
thetag = acos(FPg);      % Defasagem tensão-corrente da rede


%% Corrente de Regime na Entrada (FPg = 1)
Igr = min(roots([Rs -Vgm/2 Pl]));
vgr = (Vgm-Igr*Rs) - 1j*(Igr*Xs);
thetagr = angle(vgr);
Vgr = real(vgr);


%% Parâmetros de Simulação
h = 1E-7;                % Passo de cálculo
t = 0;                   % Tempo inicial de simulação
tf = 3;                  % Tempo final de simulação


%% Parâmetros de Gravação
tsave0 = 0.;             % Tempo inicial de gravação
tsave = tsave0;          % Tempo de gravação
npt = 200000;            % Dimensão do vetor de saída de dados

hsave = (tf-tsave0)/npt; % Passo de de gravação dos vetores de saída de dados
hsave = 1e-5;

if hsave < h             % Sendo o Passo de gravação menor que o passo de cálculo (Saturação)
    hsave = h;           % Defina o passo de gravação = passo de cálculo
    npt = (tf-tsave0)/hsave; % Recalcule o a dimensão dos vetores de saída de dados
end

n = 0;                   % Inicialização da variável de posição dos vetores de saída


%% Inicialização das variáveis
il = 0;                  % Corrente inicial no ramo l
is = 0;                  % Corrente inicial no ramo s
ig = 0;                  % Corrente inicial no ramo g

ega = 0;

vgb_ref = 0;
vgb10b_ref = 0;          % Valor inicial da tensão de polo de referência vgb10b
vgb20b_ref = 0;          % Valor inicial da tensão de polo de referência vgb20b
v2L_ref = 0;             % Tensão total inicial de referência do conversor 2L

vx = 0;                  % Grau de liberdade nas tensões de polo do conversor 3L
vy = 0;                  % Grau de liberdade entre as tensões dos conversores

Potc = 0;                % Potência de carregamento em um ciclo 60 Hz
Potd = 0;                % Potência de descarregamento em um ciclo 60 Hz

% 2L
vgb10b_int = 0;          % Valor inicial da integral da tensão no polo gb1
vgb20b_int = 0;          % Valor inicial da integral da tensão no polo gb2

ic2L_int = 0;            % Valor inicial da corrente que entra no barramento 2L

% 3L
vsa0a_int = 0;           % Valor inicial da integral da tensão no polo sa
vga0a_int = 0;           % Valor inicial da integral da tensão no polo ga
vla0a_int = 0;           % Valor inicial da integral da tensão no polo la

ic3L_int = 0;            % Valor inicial da corrente que entra no barramento 3L

% Carga
el_int = 0;

% Rede Percebida
ega_int = 0;


%% Parâmetros do controlador
% Controlador PI
Ki = 5;                 % Coeficiente integrativo
Kp = 0.2;                % Coeficiente proporcional

I0 = 3;                  % Amplitude inicial

I_error = 0;             % Erro integral
P_error = 0;             % Erro proporcional

% Controlador por Histerese
delta = Vc2L_ref*0.02;   % Histerese
delta_V2L = Vc2L_ref*0.05; % Tensão total de referência máxima do conversor 2L 

Cap = 4.4E-3;            % Capacitância de barramento

control = 1;             % Inicialização do estado da banda de histerese
mi = 0.05;               % Inclinação da reta de carregamento do conv. 3L

%% Inicialização da portadora triangular (Normalizada)
ttri = 0;           % Tempo inicial da onda triangular
ftri = 10E3;        % Frequência da onda triangular de 10 kHz
htri = 1/ftri;      % Período da onda triangular
vtri = 1/2;         % Tensão de pico da onda triangular
dtri = 1/htri;      % Derivada da onda triangular (dV/dT)
sign = -1;          % Sinal inicial da derivada (Comportamento decrescente)
atrasoPWM = 0.5*2.*pi*f_ref/ftri; % Atraso gerado no PWM

%% Extra
cont1 = 0;               % Contador 1 para debug
cont2 = 0;               % Contador 2 para debug

%% Sobretensão
sobretensao = 0;


%% ---------------- Início do Looping ----------------
while t < tf
    t = t + h;
    %% Tensão da rede
    
    if t > tf/2
        sobretensao = 1;
    end
%     sobretensao = 1;
    
    % SOBRETENSÃO
    if sobretensao == 1
        Vgm = 1.3*110*sqrt(2);
%         Vc3L_ref = 1.25*Vgm;
    else
        Vgm = 1.00*110*sqrt(2); 
    end
    
    if Vc2L_ref == Eb && sobretensao == 1 && Vc3L > 1.25*Vgm
        Vc2L_ref = Vc2L;
    end
    eg = Vgm*cos(w_ref*t);
    
    ega_ref = Vgma_ref*cos(w_ref*t); 
    
    %% Onda triangular
    if t >= ttri
        ttri = ttri + htri;
        
        %% Contolador PI da corrente da rede
        Vc3L_error = Vc3L_ref - Vc3L;
        P_error = Kp*Vc3L_error;
        I_error = I_error + htri*Ki*Vc3L_error;
        Ig = P_error + I_error + I0;

        ig_ref = Ig*cos(w_ref*t - thetag);
        ig_error = ig_ref - ig;
        
        %% Correntes de Regime Permanente
%         ig_ref = Igc*cos(w_ref*t);             
        il_ref = Il*cos(w_ref*t + thetal_ref - thetal);
        is_ref = ig_ref - il_ref;
        is_error = (is_ref  - is);
        
        %% Tensões de referência
%         vgr_ref = Vgr*cos(w_ref*t + thetagr);
%         vg_ref = eg - ig*Rs - (Ls/htri)*ig_error;
        
        vsh_ref = ega - Rs*is - (Ls/htri)*is_error;
        vel_ref = Vel_ref*cos(w_ref*t + thetal_ref + atrasoPWM);
        vse_ref = ega - vel_ref;
        
%         vsh_ref = eg - Rs*is - (Ls/htri)*is_error;
%         vel_ref = Vel_ref*cos(w_ref*t + thetal_ref + atrasoPWM);
%         vse_ref = eg - vel_ref;
        
        %% Escolha do PWM
        
        if sobretensao == 1 && Vc3L_ref < 1.25*Vgm
        %% Caso de sobretensão
            
            if Vc3L_ref < 1.25*Vgm
                Vc3L_ref = Vc3L_ref + mi;
            else
                Vc3L_ref = 1.25*Vgm;
            end
        
            %% Cálculo das tensões de polo - SOBRETENSÃO
            
            Vs = [vsh_ref,vse_ref,0];
            vxmax =  Vc3L/2 + max(Vs);
            vxmin = -Vc3L/2 + min(Vs);
            vx = (vxmax+vxmin)/2;
        
            %% Tensões de polo de referência - SOBRETENSÃO
            % 2L

            vgb_ref = eg - ega_ref;
            vgb10b_ref = vgb_ref/2;
            vgb20b_ref = -vgb_ref/2;

            % 3L
        
            vga0a_ref = vx;
            vsa0a_ref = vx - vsh_ref;
            vla0a_ref = vx - vse_ref + vgb_ref;
            
        else
        %% Caso StandBy            
            
            %% Cálculo das tensões de polo - StandBy
            Vs = [vsh_ref,vse_ref,0];
            vxmax =  Vc3L/2 + max(Vs);
            vxmin = -Vc3L/2 + min(Vs);
            vx = (vxmax+vxmin)/2;
            
            %% Controle do barramento 2L por histerese - StandyBy
            Vc2L_error = Vc2L_ref - Vc2L;

            if Vc2L_error >= delta % Carregar
                control = 1;
            elseif Vc2L_error <= -delta % Descarregar
                control = -1;
            end

            if control == 1 % Carregar
                cont1 = cont1 + 1;
                if Vc2L < Vc2L_ref
                    if ig >= 0
                        v2L_ref = delta_V2L;
                        vgb10b_ref = v2L_ref/2;
                        vgb20b_ref = -v2L_ref/2;
                    else
                        v2L_ref = -delta_V2L;
                        vgb10b_ref = v2L_ref/2;
                        vgb20b_ref = -v2L_ref/2;
                    end
                else
                    v2L_ref = 0;
                    vgb10b_ref = 1.05*Vc2L_ref;
                    vgb20b_ref = 1.05*Vc2L_ref;
                    control = 0;
                end

            elseif control == -1   % Descarregar          
                cont2 = cont2 + 1;
                if Vc2L > Vc2L_ref
                    if ig >= 0
                        v2L_ref = -delta_V2L;
                        vgb10b_ref = v2L_ref/2;
                        vgb20b_ref = -v2L_ref/2;
                    else
                        v2L_ref = delta_V2L;
                        vgb10b_ref = v2L_ref/2;
                        vgb20b_ref = -v2L_ref/2;
                    end
                else
                    v2L_ref = 0;
                    vgb10b_ref = 1.05*Vc2L_ref;
                    vgb20b_ref = 1.05*Vc2L_ref;
                    control = 0;
                end
            end
            
            %% Tensões de polo de referência - StandBy
            % 3L
            vsa0a_ref = vx - vsh_ref;
            vla0a_ref = vx - vse_ref + v2L_ref;
            vga0a_ref = vx;

        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Tensões médias nos braços
        % 2L
        vgb10b_med = vgb10b_int/htri;
        vgb20b_med = vgb20b_int/htri;

        vgb10b_int = 0;
        vgb20b_int = 0;

        % 3L
        vsa0a_med = vsa0a_int/htri;
        vga0a_med = vga0a_int/htri;
        vla0a_med = vla0a_int/htri;

        vsa0a_int = 0;
        vga0a_int = 0;
        vla0a_int = 0;
        
        % Carga
        el_med = el_int/htri;        
        el_int = 0;
        
        % Rede Percebida
        ega_med = ega_int/htri; 
        ega_int = 0;
        
        
        %% Correntes Médias nos Barramentos
        % 2L
        ic2L_med = ic2L_int/htri;
        ic2L_int = 0;
        
        % 3L
        ic3L_med = ic3L_int/htri;
        ic3L_int = 0;
        
        %% Onda triangular (Normalizada)
        if vtri <= 0
            vtri = -(1/2);
            sign = 1;
        else
            vtri = (1/2);
            sign = -1;
        end
        
    end
    %% Progressão da portadora triangular
    vtri = vtri + sign*dtri*h; % Portadora normalizada
    vt2L = Vc2L * vtri;         % Portadora Conversor 2L    
    vt3L = Vc3L * vtri;         % Portadora conversor 3L

    %% Estado das chaves
    % 2L
    if vgb10b_ref >= vt2L % Estado das chaves (Braço gb1)
        qgb1 = 1; % Chave fechada
    else
        qgb1 = 0; % Chave aberta
    end

    if vgb20b_ref >= vt2L % Estado das chaves (Braço gb2)
        qgb2 = 1; % Chave fechada
    else
        qgb2 = 0; % Chave aberta
    end

    % 3L
    if vsa0a_ref >= vt3L % Estado das chaves (Braço sa)
        qsa = 1; % Chave fechada
    else
        qsa = 0; % Chave aberta
    end

    if vga0a_ref >= vt3L % Estado das chaves (Braço ga)
        qga = 1; % Chave fechada
    else
        qga = 0; % Chave aberta
    end

    if vla0a_ref >= vt3L % Estado das chaves (Braço la)
        qla = 1; % Chave fechada 
    else
        qla = 0; % Chave aberta
    end
    
    %% Tensões de polo
    % 2L
    vgb10b = (2*qgb1 - 1)*(Vc2L/2); 
    vgb20b = (2*qgb2 - 1)*(Vc2L/2);

    % 3L
    vsa0a = (2*qsa - 1)*(Vc3L/2);
    vga0a = (2*qga - 1)*(Vc3L/2);
    vla0a = (2*qla - 1)*(Vc3L/2);
    
    %% Tensões Geradas
    vgb = vgb10b - vgb20b;
    vsh = vga0a - vsa0a;
    vse = vga0a - vla0a;
    vseb = vgb + vse;
    
    ega = eg - (vgb10b - vgb20b);
    el = ega - vse;
    
    %% Integração das tensões de polo
    % 2L
    vgb10b_int = vgb10b_int + vgb10b*h;
    vgb20b_int = vgb20b_int + vgb20b*h;

    % 3L
    vsa0a_int = vsa0a_int + vsa0a*h;
    vga0a_int = vga0a_int + vga0a*h;
    vla0a_int = vla0a_int + vla0a*h;
    
    % Carga
    el_int = el_int + el*h;
    
    % Rede percebida
    ega_int = ega_int + ega*h;

    %% Integração numérica das correntes ig e il
    il = il*(1 - h*Rl/Ll) + (h/Ll)*(ega - vse);         % Corrente do ramo l   
    is = is*(1 - h*Rs/Ls) + (h/Ls)*(ega - vsh);  % Corrente do ramo s
    ig = is + il;                               % Corrente do ramo g

    %% Modelo do barramento
    % 2L
    Rp2L = 5e3;
    Rs2L = 0.250;
    ik2L = ig*qgb1 - ig*qgb2;
    ic2L = (Rp2L*ik2L - Vc2L)/(Rp2L + Rs2L);   % Corrente barramento 2L
    Vc2L = Vc2L + (h/Cap)*ic2L;                % Tensão barramento 2L
%     Vc2L = Vc2L_ref*1.00;
    ic2L_int = ic2L_int + ic2L*h;
    
    % 3L
    Rp3L = 5e3;
    Rs3L = 0.250;
    ik3L = ig*qga - il*qla - is*qsa;
    ic3L = (Rp3L*ik3L - Vc3L)/(Rp3L + Rs3L);   % Corrente barramento 2L
    Vc3L = Vc3L + (h/Cap)*ic3L;                % Tensão barramento 3L
%     Vc3L = Vc3L_ref*1.00;
    ic3L_int = ic3L_int + ic3L*h;
    
    %% Potência nos barramentos
    % 2L
    Pot2L = ic2L*Vc2L;
    
    % 3L
    Pot3L = ic3L*Vc3L;
    
    %% Potência de carregamento e descarregamento
    if     control == 1  % Carregar
        Potc = vgb*ig;
        Potd = 0;
    
    elseif control == -1 % Descarregar
        Potd = vgb*ig;
        Potc = 0;
    
    end

    %% Salvamento das variáveis
    if tsave <= t
        %% Variáveis de salvamento
        tsave = tsave + hsave;
        n = n + 1;

        %% Tempo de simulação
        Ts(n) = t;
        
        %% Tensão da rede
        % Amplitude
        Vgms(n) = Vgm;
        
        % Senóide
        egs(n) = eg;
        
        % Rede Percebida
        egas(n)= ega;
        ega_meds(n) = ega_med; 
        
        %% Tensões de polo de referência
        vgb10b_refs(n) = vgb10b_ref;
        vgb20b_refs(n) = vgb20b_ref;

        vsa0a_refs(n) = vsa0a_ref;
        vga0a_refs(n) = vga0a_ref;
        vla0a_refs(n) = vla0a_ref;

        %% Tensões de referência
        vsh_refs(n) = vsh_ref;
        vse_refs(n) = vse_ref;
        vel_refs(n) = vel_ref;
        
%         vgba_refs(n) = vgba_ref;
%         vgb_refs(n) = vgbn_ref;
        
        v2L_refs(n) = v2L_ref;
        
        %% Tensões de polo
        vgb10bs(n) = vgb10b; 
        vgb20bs(n) = vgb20b;

        vsa0as(n) = vsa0a;
        vga0as(n) = vga0a;
        vla0as(n) = vla0a;

        %% Tensões de compensação
        vshs(n) = vsh;  % Tensão shunt 3L
        vses(n) = vse;  % Tensão série 3L
        vgbs(n) = vgb;  % Tensão série 2L
        vsebs(n)= vseb; % Tensão série 3L+2L
        els(n) = el;
        %% Tensões médias
        vgb10b_meds(n) = vgb10b_med;
        vgb20b_meds(n) = vgb20b_med;

        vsa0a_meds(n) = vsa0a_med;
        vga0a_meds(n) = vga0a_med;
        vla0a_meds(n) = vla0a_med;   
        
        el_meds(n) = el_med;
        %% Corrente de referência
        ig_refs(n) = ig_ref;
        il_refs(n) = il_ref;
        
        %% Correntes
        iss(n) = is;
        igs(n) = ig;
        ils(n) = il;

        %% Corrente e tensão nos barramentos
        % 2L
        ic2Ls(n) = ic2L;
        Vc2L_refs(n) = Vc2L_ref;
        Vc2Ls(n) = Vc2L;        

        % 3L
        ic3Ls(n) = ic3L;
        Vc3L_refs(n) = Vc3L_ref;
        Vc3Ls(n) = Vc3L;

        %% Potência no barramento dos conversores
        % 2L
        Pot2Ls(n) = Pot2L;

        % 3L
        Pot3Ls(n)= Pot3L;

        %% Portadora triangular
        vtriangles(n) = vtri;
        vt2Ls(n) = vt2L;
        vt3Ls(n) = vt3L;
        
        %% Verificação da potência de carregamento e descarregamento
        Potcs(n) = Potc;
        Potds(n) = Potd;
        
        %% Estado das chaves do conversor 2L
        qgb1s(n) = qgb1;
        qgb2s(n) = qgb2;
        
        %% Estado das chaves do conversor 3L
        qsas(n) = qsa;
        qgas(n) = qga;
        qlas(n) = qla;
        
        vgb_refs(n)=vgb_ref;
    end
end
%% ---------------- Fim do Looping ----------------
%% Simulação com parâmetro variado (Plot)
% mean(Potds);
% Vgm_s(i) = Vgm;
% FPls(i) = FPl;
% thetal_refs(i) = thetal_ref;
% FPgs(i) = FPg;
% Vls(i) = Vel_ref;
% P2s(i) = mean(Potds);

% end

% figure('name','Potência de descarregamento')
% plot(Vgm_s,P2s,'r-',[100 500],[0 0],'k-')
% plot(FPls,P2s,'r-',[0.1 1],[0 0],'k-','LineWidth',2)
% plot(thetal_refs.*180/pi,P2s,'r-',[0 110],[0 0],'k-','LineWidth',2)
% plot(FPgs,P2s,'r-',[0.1 1],[0 0],'k-','LineWidth',2)
% plot(Vls,P2s,'r-',[5 400],[0 0],'k-')

% title('Potência de Descarregamento','FontSize',18)

% xlabel("Tensão da rede (Vpico)")
% xlabel("Fator de potência da carga")
% xlabel("Diferença de fase entre Vg e Vl (º)")
% axis([0 110 -300 150])
% xlabel("Fator de potência do grid")
% xlabel("Tensão da carga (Vpico)")

% ylabel("Potência Média")
% grid minor;

%% Plots

%% Estado das chaves 2L
% figure('name','Estado das chaves 2L')
% plot(Ts,qgb1s,'r-',...
%      Ts,qgb2s,'b--','LineWidth',2),zoom
% title('Estado das chaves 2L','FontSize',18)
% legend('qgb1','qgb2','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% axis([0 tf -1.1 1.1])

%% Tensões Vsh, Vse e Vel
% figure('name','Tensão Vsh') %vsh
% plot(Ts,vsh_refs,...
%      Ts,vga0a_meds-vsa0a_meds,...
%      Ts,vshs),zoom
% title('Tensão vsh','FontSize',18)
% legend('v_{sh_{ref}}','v_{sh_med}','v_{sh_pwm}','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% axis([0 tf -(Vc3L_ref*2-20) (Vc3L_ref*2+20)])

% figure('name','Tensão Vse') %vse
% plot(Ts,vga0a_refs-vla0a_refs,...
%      Ts,vga0a_meds-vla0a_meds,...
%      Ts,vses),zoom
% title('Tensão vse','FontSize',18)
% legend('v_{se_{ref}}','vl_{se_{med}}','vl_{se_{med}}','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% axis([0 tf -Vc3L_ref-20 Vc3L_ref+20])

% figure('name','Tensão Vseb')
% plot(Ts,vel_refs,...
%      Ts,el_meds,...
%      Ts,els),zoom
% title('Tensão v_e_l','FontSize',18)
% legend('v_{el_{ref}}','v_{el_{med}}','v_{el_{pwm}}','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% axis([0 tf -Vc3L_ref-20 Vc3L_ref+20])

%% Correntes
% figure('name','Correntes')
% plot(Ts,igs,Ts,ils,Ts,iss),zoom
% title('Correntes','FontSize',18)
% legend('i_g','i_l','i_s','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Corrente (A)")

%% Tensões de referência
% figure('name','Tensões de referência')
% plot(Ts,vsh_refs,Ts,vse_refs,'r--'),zoom
% title('v*g vs v*l','FontSize',18)
% legend('vg_{ref}','vl_{ref}','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")

%% Tensões de polo
% % figure('name','Vg_ref vs modelo')
% % plot(Ts,vsh_refs,Ts,-vgb_refs + (vga0a_refs-vsa0a_refs),'r--'),zoom
% % title('v*g vs Modelo','FontSize',18)
% % legend('vg_{ref}','vg_{modelo}','FontSize',16)
% % xlabel("Tempo (s)")
% % ylabel("Tensão (V)")
% % axis([0 tf -110 110])
 
% % figure('name','Vel_ref vs modelo')
% % plot(Ts,vse_refs,Ts,vla0a_refs-vsa0a_refs,'r--'),zoom
% % title('v*l vs Modelo','FontSize',18)
% % legend('vl_{ref}','vl_{modelo}','FontSize',16)
% % xlabel("Tempo (s)")
% % ylabel("Tensão (V)")
% % axis([0 tf -110 110])
 
% % figure('name','Tensões de polo: 3L')
% % plot(Ts,vga0a_refs,Ts,vsa0a_refs,'k-',Ts,vla0a_refs,'b-'),zoom
% % title('Tensões de polo de referência: Conversor 3L','FontSize',18)
% % legend('vga0_{ref}','vsa0_{ref}','val0_{ref}','FontSize',16)
% % xlabel("Tempo (s)")
% % ylabel("Tensão (V)")
% % axis([0 tf -110 110])
 
% % figure('name','Tensões de polo: 2L')
% % plot(Ts,vgb10b_refs,Ts,vgb20b_refs,'r-'),zoom
% % title('Tensões de polo de referência: Conversor 2L','FontSize',18)
% % legend('vgb10','vgb20','FontSize',16)
% % xlabel("Tempo (s)")
% % ylabel("Tensão (V)")
% % axis([0 tf -100 100])

% ig vs vsh (Ref e Med)
% figure('name','ig vs vsh')
% yyaxis left
% plot(Ts,igs,...
%      Ts,ig_refs)
% ylabel("Corrente (A)")
% yyaxis right
% % plot(Ts,vga0a_meds-vsa0a_meds+vgb10b_meds-vgb20b_meds,...
% %      Ts,vsh_refs)
% plot(Ts,vga0a_meds-vsa0a_meds,...
%      Ts,vsh_refs)
% title('Corrente e Tensão da rede: medida e de referência','FontSize',18)
% legend('ig_{med}','ig_{ref}','vg_{med}','vg_{ref}','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% % axis([0 tf -Vc3L_ref Vc3L_ref])

%% Tensão no barramento
% figure('name','Tensão no barramento 3L')
% plot(Ts,Vc3Ls,Ts,Vc3L_refs,'r--',Ts,Vgms,'k-'),zoom
% title('Tensão do barramento 3L','FontSize',18)
% legend('Vdc3L_{med}','Vdc3L_{ref}','Vgm (amplitude)','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% grid('minor')
% 
% figure('name','Tensão no barramento 2L')
% plot(Ts,Vc2Ls,Ts,Vc2L_refs,'r--',...
%     [tsave0 tf],[Vc2L_ref+delta Vc2L_ref+delta],'k-',...
%     [tsave0 tf],[Vc2L_ref-delta Vc2L_ref-delta],'k-'),zoom
% title('Tensão do barramento 2L ~ Delta = 5%Vdc2L_{ref}','FontSize',18)
% legend('Vdc2L_{med}','Vdc2L_{ref}','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% grid('minor')
%%
% figure('Name','ig: Correntes de referência e atuais')
% plot(Ts,ig_refs,Ts,igs)
% 
% figure('Name','il: Correntes de referência e atuais')
% plot(Ts,il_refs,Ts,ils)

% figure(16)
% plot(Ts,vsh_refs,Ts,vg_ref_es)
% legend('controlador','calculado')
% figure(17)
% plot(Ts,ig_refs)
% figure(18)
% plot(Ts,Igs,Ts,ic2L_meds,Ts,ic3L_meds,[tsave0 tf],[Ig_e Ig_e])

%% Rede e Tensão Percebida pelo 3L 
% figure('name','Rede e Tensão Percebida pelo 3L')
% plot(Ts,egs,Ts,egas,'r--',Ts,ega_meds,'k-'),zoom
% title('Tensão da Rede e Percebida pelo 3L','FontSize',18)
% legend('E_g','E_{ga}','E_{ga_{med}}','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% grid('minor')

%% THD

% wthd_vg = wthdf(vshs, 1/h, f_ref)
% wthd_vl = wthdf(vses, 1/h, f_ref)
% 
% thd_ig = thdf(igs, 1/h, f_ref)
% thd_il = thdf(ils, 1/h, f_ref)

%% Média de chaveamento
m_qgb1 = mean(qgb1s);
m_qgb2 = mean(qgb2s);

%% Figuras do Relatório Parcial
% Barramentos

ax1 = [0 3 150 300];
sp1 = subplot(3,1,1);
plot(Ts,Vc3Ls,Ts,Vc3L_refs,'r--'),zoom
title('Barramento $E_a$','FontSize',18,'Interpreter','latex')
legend('$E_a$','$E_a^*$','FontSize',16,'Interpreter','latex','Location','northwest')
xlabel('Tempo (s)','FontSize',14)
ylabel('Tensão (V)','FontSize',14)
grid('minor')
axis(ax1);

ax2 = [0 3 150 300];
sp2 = subplot(3,1,2);
plot(Ts,Vc2Ls,Ts,Vc2L_refs,'r--',...
    [tsave0 tf/2],[190+delta 190+delta],'k-',...
    [tsave0 tf/2],[190-delta 190-delta],'k-',...
    [1.63 tf],[Vc2L_ref+delta Vc2L_ref+delta],'k-',...
    [1.63 tf],[Vc2L_ref-delta Vc2L_ref-delta],'k-'),zoom
title('Barramento $E_b$','FontSize',18,'Interpreter','latex')
legend('$E_b$','$E_b^*$','FontSize',16,'Interpreter','latex','Location','northwest')
xlabel('Tempo (s)','FontSize',14)
ylabel('Tensão (V)','FontSize',14)
grid('minor')
axis(ax2);

ax3 = [0 3 -200 200];
sp3 = subplot(3,1,3);
plot(Ts,vgb10b_meds-vgb20b_meds,'b-'),zoom
title('Tens\~{a}o $v_{sb}$','FontSize',18,'Interpreter','latex')
legend('$v_{sb}$','FontSize',16,'Interpreter','latex','Location','northwest')
xlabel('Tempo (s)','FontSize',14)
ylabel('Tensão (V)','FontSize',14)
grid('minor')
axis(ax3);



% Eg vs Ig
figure('name','Tensão e Corrente: Grid') %vsh

yyaxis right
plot(Ts,igs,'LineWidth',1)
ylabel("Corrente [A]",'FontSize',16)
axis([1 2 -35 35])

yyaxis left
plot(Ts,egs,'LineWidth',1)
ylabel("Tensão [V]",'FontSize',16)
axis([1 2 -300 300])

title('$e_g$ vs $i_g$','FontSize',24,'Interpreter','latex')
legend('$e_g$','$i_g$','FontSize',24,'Interpreter','latex','Location','northwest')
xlabel("Tempo [s]",'FontSize',16)
grid minor



% El vs Il
figure('name','Tensão e Corrente: Load') %vsh

yyaxis right
plot(Ts,ils,'LineWidth',1)
ylabel("Corrente [A]",'FontSize',16)
axis([1 2 -35 35])

yyaxis left
plot(Ts,el_meds,'LineWidth',1)
ylabel("Tensão [V]",'FontSize',16)
axis([1 2 -300 300])

title('$e_l$ vs $i_l$','FontSize',24,'Interpreter','latex')
legend('$e_l$','$i_l$','FontSize',24,'Interpreter','latex','Location','northwest')
xlabel("Tempo [s]",'FontSize',16)
grid minor