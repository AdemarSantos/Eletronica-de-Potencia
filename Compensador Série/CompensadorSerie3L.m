%% Compensador série 3L (1L em standby)

clear;close all;format long;clc;

%% Simulação com parâmetro variado (Sel. parâmetro)
% i = 0;
% for Vgm = 100:1:500
% for FPl = 0.1:0.005:1
% for alpha = 0:0.5*pi/180:110*pi/180
% for FPg = 0.1:0.005:1
% for Vl_ref = 5:1:400
%     i = i+1;

%% ---------------- Condições do Sistema ----------------------------
SwSg = 1;
Vgm = SwSg*110*sqrt(2);  % Tensão de pico da rede (Vgm)
Vgm_in = Vgm;
Vl_ref = 110*sqrt(2);    % Tensão de pico de referência braço L

f_ref = 60;              % Frequência de referência
w_ref = 2*pi*f_ref;      % Frequência angular de referência

% Barramento 3L
Vc3L_ref = 180;          % Tensão de referência do barramento 3L (Conv. A)
Vc3L = 0;                % Tensão medida no barramento 3L (Conv. A)          
Ea = Vc3L_ref; 


% Defasagem entre Vg e Vl
alpha_limite = acos(((2*Ea^2-Ea^2)/(2*Ea^2)));
alpha = 0*(pi/180);      % Fase da tensão de referência braço L

%% ---------------- Impedâncias de entrada e saída ------------------
Pl = 1000;
FPl = 0.8; % Ind.

thetal = acos(FPl);
Nl = Pl/FPl;
Ql = Nl*sin(thetal);
Sl = Pl + 1j*Ql;

Zl = (1/2)*(Vl_ref^2)/conj(Sl); %(Vl_ref Tensão de pico)
Rl = real(Zl);           % Resistência da carga
Xl = imag(Zl);
Ll = Xl/w_ref;           % Indutância da carga

Il = Vl_ref/abs(Zl);

Rh = 0.2;
Lh = 5e-3;
Xh = Lh*w_ref;
thetag = 0;
% thetag = acos(FPg);

%% ---------------- Corrente de regime na entrada ------------------
Igr = min(roots([Rh -Vgm/2 Pl]));
vgr = (Vgm-Igr*Rh) - 1j*(Igr*Xh);
thetagr = angle(vgr);
Vgr = real(vgr);

%% ---------------- Parâmetros de Simulação ------------------
h = 1E-6;                % Passo de cálculo
t = 0;                   % Tempo inicial de simulação
tf = 2;

%% ---------------- Parâmetros de Gravação ------------------
% tsave0 = tf-1/f_ref;     % Tempo inicial de gravação
tsave0 = 0;              % Tempo inicial de gravação
tsave = tsave0;          % Tempo de gravação
npt = 50000;             % Dimensão do vetor de saída de dados

hsave = (tf-tsave0)/npt; % Passo de de gravação dos vetores de saída de dados

if hsave < h             % Sendo o Passo de gravação menor que o passo de cálculo (Saturação)
    hsave = h;           % Defina o passo de gravação = passo de cálculo
    npt = (tf-tsave0)/hsave; % Recalcule o a dimensão dos vetores de saída de dados
end
hsave = h;
n = 0;                   % Inicialização da variável de posição dos vetores de saída

%% ---------------- Inicialização das variáveis ------------------
ig = 0;                  % Corrente inicial no ramo g
il = 0;                  % Corrente inicial no ramo l
ih = 0;

vgb10b_int = 0;          % Valor inicial da integral da tensão no polo gb1
vgb20b_int = 0;          % Valor inicial da integral da tensão no polo gb2
vha0a_int = 0;           % Valor inicial da integral da tensão no polo ha
vga0a_int = 0;           % Valor inicial da integral da tensão no polo ga
vla0a_int = 0;           % Valor inicial da integral da tensão no polo la

ic2L_int = 0;            % Valor inicial da corrente que entra no barramento 2L
ic3L_int = 0;            % Valor inicial da corrente que entra no barramento 3L

vx = 0;                  % Grau de liberdade nas tensões de polo do conversor 3L
vy = 0;                  % Grau de liberdade entre as tensões dos conversores

Potc = 0;                % Potência de carregamento em um ciclo 60 Hz
Potd = 0;                % Potência de descarregamento em um ciclo 60 Hz
control = -1;            % Estado da banda de histerese

u = 0.5;                 % Fator de repartição do PWM

Sg = 0;
Sw = 0;

%% Extra
vgb = 0;
cont1 = 0;
cont2 = 0;
delta = 0.05;

%% ---------------- Parâmetros do controlador ------------------
Ki = 15;                 % Coeficiente integrativo
Kp = 0.2;                % Coeficiente proporcional
I0 = 3;                  % Amplitude inicial

I_error = 0;             % Erro integral
P_error = 0;             % Erro proporcional

Cap = 4.4E-3;            % Capacitância de barramento

%% ---------------- Inicialização da portadora triangular ------------------
ttriangle = 0;           % Tempo inicial da onda triangular
ftriangle = 10E3;        % Frequência da onda triangular de 10 kHz
htriangle = 1/ftriangle; % Período da onda triangular
Vt = 1/2;         % Tensão de pico da onda triangular
vtriangle = Vt;          % Tensão máxima da onda triangular
dtriangle = 2*Vt/htriangle;% Derivada da onda triangular (dV/dT)
sign = -1;               % Sinal inicial da derivada (Comportamento decrescente)


%% Estado de operação
PWM_type = 1; % (2L=1;3L=0)

%% ---------------- Início do Looping ----------------
while t < tf
    t = t + h;
    %% Tensão da rede

    if t > 1*tf/3
        Vgm = Vgm_in*1.5;
    end
    
    eg = Vgm*cos(w_ref*t);   
    
    if Vgm < (1-delta)*Vl_ref
        PWM_type = 0;
        Sg = 1;
    elseif  Vgm > (1+delta)*Vl_ref
        Sw = 1;
    end
    
    if Sg == 1 && Vgm > (1-delta)*Vl_ref
        Sg = 0;
     
    elseif Sw == 1 && Vgm < (1+delta)*Vl_ref
        Sw = 0;
    
    end
     
    
    %% Onda triangular
    if t >= ttriangle
        ttriangle = ttriangle + htriangle;
        %% Contolador PI da corrente da rede
        Vc3L_error = Vc3L_ref - Vc3L;
        P_error = Kp*Vc3L_error;
        I_error = I_error + htriangle*Ki*Vc3L_error;
        Ig = P_error + I_error + I0;

        
%         ig_error = ig_ref - ig;

        %% Correntes de Referência do controlador
        ig_ref = Ig*cos(w_ref*t - thetag);
        il_ref = Il*cos(w_ref*t + alpha - thetal);
        
        ih_ref = ig_ref-il_ref;
        ih_error = ih_ref - ih;
        
        %% Tensões de referência
        
        % CONTROLE PREDITIVO
%       vg_ref = Vgr*cos(w_ref*t + thetagr);
%       vg_ref = eg - ig*Rg - (Lg/htriangle)*ig_error;
        vg_ref = eg - ih*Rh - (Lh/htriangle)*ih_error;
        vl_ref = Vl_ref*cos(w_ref*t + alpha);
        
        vse_ref = eg - vl_ref;
        
        %% Verificação do PWM
        if Sw == 1 && Vc3L > Vgm
            Vc3L_ref = 1.05*Vgm;
            PWM_type = 0;
        elseif Sw == 0
            Vc3L_ref = Ea;
        end
        
        
        %% Cálculo das tensões de polo
        if PWM_type == 1 % 2L
            Vs = [vse_ref/2,-vse_ref/2];
            
            vxmax = Vc3L_ref/2 - max(Vs);
            vxmin = -Vc3L_ref/2 - min(Vs);
            
            vx = u*vxmax + (1-u)*vxmin;
            
            vga0a_ref =  vse_ref/2 + vx;
            vla0a_ref = -vse_ref/2 + vx;
            vha0a_ref = 1.05*Vc3L_ref;
        
        else % 3L
            Vs = [vg_ref,vse_ref,0];
            vxmax =  Vc3L_ref/2 + max(Vs);
            vxmin = -Vc3L_ref/2 + min(Vs);
            
            vx = u*vxmax + (1-u)*vxmin;
            
            vga0a_ref = vx;
            vla0a_ref = vga0a_ref - vse_ref ;
            vha0a_ref = vga0a_ref - vg_ref;
        end

        %% Tensões médias nos braços
        % 3L
        vha0a_med = vha0a_int/htriangle;
        vga0a_med = vga0a_int/htriangle;
        vla0a_med = vla0a_int/htriangle;

        vha0a_int = 0;
        vga0a_int = 0;
        vla0a_int = 0;
        
        ic3L_med = ic3L_int/htriangle;
        ic3L_int = 0;
        
        %% Portadora triangular
        if vtriangle <= 0
           vtriangle = -(Vt);
           sign = 1;
        else
            vtriangle = (Vt);
            sign = -1;
        end
    end
    %% Progressão da portadora triangular
    vtriangle = vtriangle + sign*dtriangle*h;
    vtri = vtriangle*Vc3L;

    %% Estado das chaves
    % 3L
    if vha0a_ref >= vtri % Estado das chaves (Braço ha)
        qha = 1; % Chave fechada
    else
        qha = 0; % Chave aberta
    end

    if vga0a_ref >= vtri % Estado das chaves (Braço ga)
        qga = 1; % Chave fechada
    else
        qga = 0; % Chave aberta
    end

    if vla0a_ref >= vtri % Estado das chaves (Braço la)
        qla = 1; % Chave fechada 
    else
        qla = 0; % Chave aberta
    end

    %% Tensões de polo
    % 3L
    vha0a = (2*qha - 1)*(Vc3L/2);
    vga0a = (2*qga - 1)*(Vc3L/2);
    vla0a = (2*qla - 1)*(Vc3L/2);

    %% Integração das tensões de polo
    % 3L
    vha0a_int = vha0a_int + vha0a*h;
    vga0a_int = vga0a_int + vga0a*h;
    vla0a_int = vla0a_int + vla0a*h;

    %% Tensões de fase
    vg = vga0a - vha0a;
    vl = vla0a - vha0a;
    vse = vga0a - vla0a;
    
    %% Integração numérica das correntes ig e il
    if PWM_type == 1
        il = il*(1 - h*Rl/Ll) + (h/Ll)*(eg - vse);
        ig = il;
        ih = 0;
    else
        ih = ih*(1 - h*Rh/Lh) + (h/Lh)*(eg - vg);  % Corrente do ramo h
        il = il*(1 - h*Rl/Ll) + (h/Ll)*(eg - vse); % Corrente do ramo l   
        ig = il + ih;
    end
    %% Controle do barramento
    % 3L
    ic3L = ig*qga - il*qla - ih*qha;
    Vc3L = Vc3L + (h/Cap)*ic3L;
%     Vc3L = Vc3L_ref*1.00;
    ic3L_int = ic3L_int + ic3L*h;
    
    %% Potência no barramento
    % 3L
    Pot3L = ic3L*Vc3L;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Salvamento das variáveis
    if tsave <= t
        %% Variáveis de salvamento
        tsave = tsave + hsave;
        n = n + 1;

        %% Tempo de simulação
        Ts(n) = t;
        
        %% Rede
        egs(n) = eg;
        
        %% Tensões de polo de referência
        vha0a_refs(n) = vha0a_ref;
        vga0a_refs(n) = vga0a_ref;
        vla0a_refs(n) = vla0a_ref;
        
        %% Estado das chaves
        qhas(n) = qha;
        qgas(n) = qga;
        qlas(n) = qla;
        
        %% Tensões de referência
        vg_refs(n) = vg_ref;
        vl_refs(n) = vl_ref;

        %% Tensões de polo
        vha0as(n) = vha0a;
        vga0as(n) = vga0a;
        vla0as(n) = vla0a;

        %% Tensões de fase
        vgs(n) = vg;
        vls(n) = vl;
        vses(n) = vse;

        %% Tensões médias

        vha0a_meds(n) = vha0a_med;
        vga0a_meds(n) = vga0a_med;
        vla0a_meds(n) = vla0a_med;   

        %% Corrente de referência
        ig_refs(n) = ig_ref;
        il_refs(n) = il_ref;
        
        %% Correntes
        ihs(n) = ih;
        igs(n) = ig;
        ils(n) = il;

        %% Corrente e tensão nos barramentos     
        % 3L
        ic3Ls(n) = ic3L;
        Vc3L_refs(n) = Vc3L_ref;
        Vc3Ls(n) = Vc3L;

        %% Potência no barramento dos conversores
        % 3L
        Pot3Ls(n)= Pot3L;

        %% Portadora triangular
        vtriangles(n) = vtriangle;
        vtris(n) = vtri;
    end
end
%% ---------------- Fim do Looping ----------------
%% Simulação com parâmetro variado (Plot)
% mean(Potds);
% Vgms(i) = Vgm;
% FPls(i) = FPl;
% alphas(i) = alpha;
% FPgs(i) = FPg;
% Vls(i) = Vl_ref;
% P2s(i) = mean(Potds);

% end

% figure('name','Potência de descarregamento')
% plot(Vgms,P2s,'r-',[100 500],[0 0],'k-')
% plot(FPls,P2s,'r-',[0.1 1],[0 0],'k-','LineWidth',2)
% plot(alphas.*180/pi,P2s,'r-',[0 110],[0 0],'k-','LineWidth',2)
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

%% Tensões Vg e Vl
figure('name','Tensão Vg') %vg
plot(Ts,vg_refs,...
     Ts,vga0a_meds-vha0a_meds,...
     Ts,vgs),zoom
title('Tensão vg','FontSize',18)
legend('vg_{ref}','vg_{med}','vg_{pwm}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")
% axis([0 tf -(Vc3L_ref*2-20) (Vc3L_ref*2+20)])

figure('name','Tensão el') %el
plot(Ts,vl_refs,...
     Ts,egs - (vga0a_meds-vla0a_meds),...
     Ts,egs - (vses),...
     Ts,egs),zoom
title('Tensão el','FontSize',18)
legend('el_{ref}','el_{med}','el_{pwm}','eg','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")
% axis([0 tf -Vc3L_ref-20 Vc3L_ref+20])

%% Correntes
figure('name','Correntes')
plot(Ts,igs,Ts,ils,Ts,ihs),zoom
title('Correntes','FontSize',18)
legend('ig','il','ih','FontSize',16)
xlabel("Tempo (s)")
ylabel("Corrente (A)")

%% Tensões de referência
% figure('name','Tensões de referência')
% plot(Ts,vg_refs,Ts,vl_refs,'r--'),zoom
% title('v*g vs v*l','FontSize',18)
% legend('vg_{ref}','vl_{ref}','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")

%% Tensões de polo
% % figure(4)
% % plot(Ts,vg_refs,Ts,-vgb_refs + (vga0a_refs-vha0a_refs),'r--'),zoom
% % title('v*g vs Modelo','FontSize',18)
% % legend('vg_{ref}','vg_{modelo}','FontSize',16)
% % xlabel("Tempo (s)")
% % ylabel("Tensão (V)")
% % axis([0 tf -110 110])
 
% % figure(5)
% % plot(Ts,vl_refs,Ts,vla0a_refs-vha0a_refs,'r--'),zoom
% % title('v*l vs Modelo','FontSize',18)
% % legend('vl_{ref}','vl_{modelo}','FontSize',16)
% % xlabel("Tempo (s)")
% % ylabel("Tensão (V)")
% % axis([0 tf -110 110])
 
% % figure(6)
% % plot(Ts,vga0a_refs,Ts,vha0a_refs,'k-',Ts,vla0a_refs,'b-'),zoom
% % title('Tensões de polo de referência: Conversor 3L','FontSize',18)
% % legend('vga0_{ref}','vha0_{ref}','val0_{ref}','FontSize',16)
% % xlabel("Tempo (s)")
% % ylabel("Tensão (V)")
% % axis([0 tf -110 110])
 
% % figure(7)
% % plot(Ts,vgb10b_refs,Ts,vgb20b_refs,'r-'),zoom
% % title('Tensões de polo de referência: Conversor 2L','FontSize',18)
% % legend('vgb10','vgb20','FontSize',16)
% % xlabel("Tempo (s)")
% % ylabel("Tensão (V)")
% % axis([0 tf -100 100])

%% ig vs vg (Ref e Med)
% figure('name','ig vs vg')
% yyaxis left
% plot(Ts,igs,...
%      Ts,ig_refs)
% ylabel("Corrente (A)")
% yyaxis right
% plot(Ts,vga0a_meds-vha0a_meds,...
%      Ts,vg_refs)
% title('Corrente e Tensão da rede: medida e de referência','FontSize',18)
% legend('ig_{med}','ig_{ref}','vg_{med}','vg_{ref}','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% % axis([0 tf -Vc3L_ref Vc3L_ref])

%% Tensão no barramento
figure('name','Tensão no barramento 3L')
plot(Ts,Vc3Ls,Ts,Vc3L_refs,'r-',Ts,egs),zoom
title('Tensão do barramento 3L','FontSize',18)
legend('Vdc3L_{med}','Vdc3L_{ref}','e_{g}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")

%% Tensão de polo de referência, Chaveamento e triangular
figure('name','RefPolTri g')
plot(Ts,vga0a_refs,...
     Ts,qgas.*100,...
     Ts,vtris,'r-'),zoom
title('Teste PWM G','FontSize',18)
legend('v_{g0_{Ref}}','q_{ga}','Portadora','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")

figure('name','RefPolTri l')
plot(Ts,vla0a_refs,...
     Ts,qlas.*100,...
     Ts,vtris,'r-'),zoom
title('Teste PWM L','FontSize',18)
legend('v_{l0_{Ref}}','q_{la}','Portadora','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")

%%
% figure('Name','ig: Correntes de referência e atuais')
% plot(Ts,ig_refs,Ts,igs)
% 
% figure('Name','il: Correntes de referência e atuais')
% plot(Ts,il_refs,Ts,ils)

% figure(16)
% plot(Ts,vg_refs,Ts,vg_ref_es)
% legend('controlador','calculado')
% figure(17)
% plot(Ts,ig_refs)
% figure(18)
% plot(Ts,Igs,Ts,ic2L_meds,Ts,ic3L_meds,[tsave0 tf],[Ig_e Ig_e])

%% THD

% wthd_vg = wthdf(vgs, 1/h, f_ref)
% wthd_vl = wthdf(vls, 1/h, f_ref)
% 
% thd_ig = thdf(igs, 1/h, f_ref)
% thd_il = thdf(ils, 1/h, f_ref)