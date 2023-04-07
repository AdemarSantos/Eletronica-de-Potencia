%% Conversor 5L2D com controle da corrente da rede e dos barramentos

clear;
close all;
format long;
clc;

%% Simulação com parâmetro variado (Sel. parâmetro)
i = 0;
% for Vgm = 100:1:500
% for FPl = 0.1:0.005:1
% for alpha = 0:0.5*pi/180:110*pi/180
% for FPg = 0.1:0.005:1
% for Vl_ref = 5:1:400
%     i = i+1;

%% ---------------- Condições do Sistema ----------------------------
Vgm = 220*sqrt(2);       % Tensão de pico da rede (Vgm)
Vl_ref = 220*sqrt(2);    % Tensão de pico de referência braço L

f_ref = 60;              % Frequência de referência
w_ref = 2*pi*f_ref;      % Frequência angular de referência

% Barramento 3L
Vc3L_ref = 340;          % Tensão de referência do barramento 3L (Conv. A)
Vc3L = 300;         % Tensão medida no barramento 3L (Conv. A)          
Ea = Vc3L_ref; 

% Barramento 2L
Vc2L_ref = 340;          % Tensão de referência do barramento 2L (Conv. B)
Vc2L = 300;         % Tensão medida no barramento 2L (Conv. B) 
Eb = Vc2L_ref;

% Defasagem entre Vg e Vl
alpha_limite = acos( (Eb^2)/(2*(Ea+Eb)*(Eb)) ); 
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

Rg = 0.2;
Lg = 5e-3;
Xg = Lg*w_ref;
thetag = 0;
% thetag = acos(FPg);
%% ---------------- Corrente de regime na entrada ------------------
Igr = min(roots([Rg -Vgm/2 Pl]));
vgr = (Vgm-Igr*Rg) - 1j*(Igr*Xg);
thetagr = angle(vgr);
Vgr = real(vgr);

%% ---------------- Parâmetros de Simulação ------------------
h = 1E-7;                % Passo de cálculo
t = 0;                   % Tempo inicial de simulação
tf = 10;

%% ---------------- Parâmetros de Gravação ------------------
tsave0 = 0;     % Tempo inicial de gravação
tsave = tsave0;          % Tempo de gravação
npt = 1000000;             % Dimensão do vetor de saída de dados

hsave = (tf-tsave0)/npt; % Passo de de gravação dos vetores de saída de dados

if hsave < h             % Sendo o Passo de gravação menor que o passo de cálculo (Saturação)
    hsave = h;           % Defina o passo de gravação = passo de cálculo
    npt = (tf-tsave0)/hsave; % Recalcule o a dimensão dos vetores de saída de dados
end
% hsave = h;
n = 0;                   % Inicialização da variável de posição dos vetores de saída

%% ---------------- Inicialização das variáveis ------------------
ig = 0;                  % Corrente inicial no ramo g
il = 0;                  % Corrente inicial no ramo l

vgb10b_int = 0;          % Valor inicial da integral da tensão no polo gb1
vgb20b_int = 0;          % Valor inicial da integral da tensão no polo gb2
vha0a_int = 0;           % Valor inicial da integral da tensão no polo ha
vga0a_int = 0;           % Valor inicial da integral da tensão no polo ga
vla0a_int = 0;           % Valor inicial da integral da tensão no polo la

vgb10b_ref = 0;
vgb20b_ref = 0;

ic2L_int = 0;            % Valor inicial da corrente que entra no barramento 2L
ic3L_int = 0;            % Valor inicial da corrente que entra no barramento 3L

vx = 0;                  % Grau de liberdade nas tensões de polo do conversor 3L
vy = 0;                  % Grau de liberdade entre as tensões dos conversores

Potc = 0;                % Potência de carregamento em um ciclo 60 Hz
Potd = 0;                % Potência de descarregamento em um ciclo 60 Hz
control = 1;             % Estado da banda de histerese

%% Extra
vgb = 0;
cont1 = 0;
cont2 = 0;

%% ---------------- Parâmetros do controlador ------------------
Ki = 15;                 % Coeficiente integrativo
Kp = 0.2;                % Coeficiente proporcional
I0 = 3;                  % Amplitude inicial

I_error = 0;             % Erro integral
P_error = 0;             % Erro proporcional

delta = Vc2L_ref*0.02;   % Histerese
delta2 = Vc2L_ref*0.05;

v2L_ref = 0;

Cap = 4.4E-3;            % Capacitância de barramento

%% ---------------- Inicialização da portadora triangular ------------------
ttriangle = 0;           % Tempo inicial da onda triangular
ftriangle = 10E3;        % Frequência da onda triangular de 10 kHz
htriangle = 1/ftriangle; % Período da onda triangular
Vt = Vc3L_ref/2;         % Tensão de pico da onda triangular
vtriangle = Vt;          % Tensão máxima da onda triangular
dtriangle = 2*Vt/htriangle;% Derivada da onda triangular (dV/dT)
sign = -1;               % Sinal inicial da derivada (Comportamento decrescente)

%% ---------------- Início do Looping ----------------
while t < tf
    t = t + h;
    %% Tensão da rede
    eg = Vgm*cos(w_ref*t);
    
%     if t >= tf/2
%        Rg = 2; 
%     end
    
    %% Onda triangular
    if t >= ttriangle
        ttriangle = ttriangle + htriangle;
        %% Contolador PI da corrente da rede
        Vc3L_error = Vc3L_ref - Vc3L;
        P_error = Kp*Vc3L_error;
        I_error = I_error + htriangle*Ki*Vc3L_error;
        Ig = P_error + I_error + I0;

        ig_ref = Ig*cos(w_ref*t - thetag);
        ig_error = ig_ref - ig;

        %% Tensões de referência
        vgr_ref = Vgr*cos(w_ref*t + thetagr);
        vg_ref = eg - ig*Rg - (Lg/htriangle)*ig_error;
        vl_ref = Vl_ref*cos(w_ref*t + alpha);
        
        %% Corrente il
        il_ref = Il*cos(w_ref*t + alpha - thetal);
        
        %% Cálculo das tensões de polo
%         vxmaxgba =  (Vc2L+Vc3L/2) - vg_ref;
%         vxmingba = -(Vc2L+Vc3L/2) - vg_ref;
%         vxmaxl0  =  Vc3L/2 - max([vl_ref,0]);
%         vxminl0  = -Vc3L/2 - min([vl_ref,0]);
%         vxmax = min([vxmaxgba,vxmaxl0]);
%         vxmin = max([vxmingba,vxminl0]);
         
        Vs = [vg_ref,vl_ref,0];
        vxmax =  Vc3L/2 - max(Vs);
        vxmin = -Vc3L/2 - min(Vs);

        % Controle da variável Vx de modo a diminuir a tensão sobre Vgb
%         if vg_ref >= 0
%             vx = vxmin;
%         else
%             vx = vxmax;
%         end
        
        vx = (vxmax+vxmin)/2;

%         vgba_ref  = vg_ref + vx;

%         vymaxga0 =  Vc3L/2 - vgba_ref/2;
%         vyminga0 = -Vc3L/2 - vgba_ref/2;
%         vymaxgb  =  Vc2L + vgba_ref/2;
%         vymingb  = -Vc2L + vgba_ref/2;
%         vymax = min([vymaxga0,vymaxgb]);
%         vymin = max([vyminga0,vymingb]);

        %% Controle do barramento 2L por histerese
        Vc2L_error = Vc2L_ref - Vc2L;

%         if Vc2L_error >= delta % Carregar
%             control = 1;
%         elseif Vc2L_error <= -delta % Descarregar
%             control = -1;
%         end
        
        if eg > -0.25*Vgm && eg < 0.25*Vgm
            if Vc2L_error >= delta % Carregar
                cont1 = cont1 + 1;
                if ig >= 0
                    v2L_ref = delta2;
                    vgb10b_ref = v2L_ref/2;
                    vgb20b_ref = -v2L_ref/2;
                else
                    v2L_ref = -delta2;
                    vgb10b_ref = v2L_ref/2;
                    vgb20b_ref = -v2L_ref/2;
                end

            elseif Vc2L_error <= -delta   % Descarregar          
                cont2 = cont2 + 1;
                if ig >= 0
                    v2L_ref = -delta2;
                    vgb10b_ref = v2L_ref/2;
                    vgb20b_ref = -v2L_ref/2;
                else
                    v2L_ref = delta2;
                    vgb10b_ref = v2L_ref/2;
                    vgb20b_ref = -v2L_ref/2;
                end
                
            else
                vgb10b_ref = 1.05*Vc2L_ref;
                vgb20b_ref = 1.05*Vc2L_ref; 
            end
        end
        
%         vgbn_ref = -vgba_ref/2 + vy;
        %% Tensões de polo de referência
        % 2L
%         vgb10b_ref = -vgbn_ref/2;
%         vgb20b_ref = vgbn_ref/2;
          
%           vgb10b_ref = v2L_ref/2;
%           vgb20b_ref = -v2L_ref/2;
          
        % 3L
%         vga0a_ref = vgba_ref/2 + vy;
%         vx = 0;
        vga0a_ref = vg_ref + vx;
        vla0a_ref = vl_ref + vx;
        vha0a_ref = vx;

        %% Tensões médias nos braços
        % 2L
        vgb10b_med = vgb10b_int/htriangle;
        vgb20b_med = vgb20b_int/htriangle;

        vgb10b_int = 0;
        vgb20b_int = 0;

        ic2L_med = ic2L_int/htriangle;
        ic2L_int = 0;

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

    %% Estado das chaves
    % 2L
    if vgb10b_ref >= vtriangle % Estado das chaves (Braço gb1)
        qgb1 = 1; % Chave fechada
    else
        qgb1 = 0; % Chave aberta
    end

    if vgb20b_ref >= vtriangle % Estado das chaves (Braço gb2)
        qgb2 = 1; % Chave fechada
    else
        qgb2 = 0; % Chave aberta
    end

    % 3L
    if vha0a_ref >= vtriangle % Estado das chaves (Braço ha)
        qha = 1; % Chave fechada
    else
        qha = 0; % Chave aberta
    end

    if vga0a_ref >= vtriangle % Estado das chaves (Braço ga)
        qga = 1; % Chave fechada
    else
        qga = 0; % Chave aberta
    end

    if vla0a_ref >= vtriangle % Estado das chaves (Braço la)
        qla = 1; % Chave fechada 
    else
        qla = 0; % Chave aberta
    end
    
    %% Tensões de polo
    % 2L
    vgb10b = (2*qgb1 - 1)*(Vc2L/2); 
    vgb20b = (2*qgb2 - 1)*(Vc2L/2);

    % 3L
    vha0a = (2*qha - 1)*(Vc3L/2);
    vga0a = (2*qga - 1)*(Vc3L/2);
    vla0a = (2*qla - 1)*(Vc3L/2);

    %% Integração das tensões de polo
    % 2L
    vgb10b_int = vgb10b_int + vgb10b*h;
    vgb20b_int = vgb20b_int + vgb20b*h;

    % 3L
    vha0a_int = vha0a_int + vha0a*h;
    vga0a_int = vga0a_int + vga0a*h;
    vla0a_int = vla0a_int + vla0a*h;

    %% Tensões de fase
%     vgha = vga0a - vha0a;
    vgb = vgb10b - vgb20b;

%     vg = vgb + vgha;
    vg = vga0a - vha0a;
    vl = vla0a - vha0a;

    %% Integração numérica das correntes ig e il
    ig = ig*(1 - h*Rg/Lg) + (h/Lg)*(eg - vg);  % Corrente do ramo g
    il = il*(1 - h*Rl/Ll) + (h/Ll)*vl;         % Corrente do ramo l   
    ih = ig - il;

    %% Controle do barramento
    % 2L
    Rp2L = 1e7;
    Rs2L = 0.2;
    
%     if t >= tf/2
%        Rp2L = 1e5; 
%     end
    
    
    ik2L = ig*qgb1 - ig*qgb2;
    ic2L = (Rp2L*ik2L - Vc2L)/(Rp2L + Rs2L);
    Vc2L = Vc2L + (h/Cap)*ic2L;
%     Vc2L = Vc2L_ref*1.00;

    ic2L_int = ic2L_int + ic2L*h;
    
    % 3L
    Rp3L = 1e9;
    Rs3L = 0;
    
    ik3L = ig*qga - il*qla - ih*qha;
    ic3L = (Rp3L*ik3L - Vc3L)/(Rp3L + Rs3L);
    Vc3L = Vc3L + (h/Cap)*ic3L;
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
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Salvamento das variáveis
    if tsave <= t
        %% Variáveis de salvamento
        tsave = tsave + hsave;
        n = n + 1;

        %% Tempo de simulação
        Ts(n) = t;

        %% Tensões de polo de referência
        vgb10b_refs(n) = vgb10b_ref;
        vgb20b_refs(n) = vgb20b_ref;

        vha0a_refs(n) = vha0a_ref;
        vga0a_refs(n) = vga0a_ref;
        vla0a_refs(n) = vla0a_ref;

        %% Tensões de referência
        vg_refs(n) = vg_ref;
        vl_refs(n) = vl_ref;

%         vgba_refs(n) = vgba_ref;
%         vgb_refs(n) = vgbn_ref;
        v2L_refs(n) = v2L_ref;
        %% Tensões de polo
        vgb10bs(n) = vgb10b; 
        vgb20bs(n) = vgb20b;

        vha0as(n) = vha0a;
        vga0as(n) = vga0a;
        vla0as(n) = vla0a;

        %% Tensões de fase
        vgs(n) = vg;
        vls(n) = vl;    

        %% Tensões médias
        vgb10b_meds(n) = vgb10b_med;
        vgb20b_meds(n) = vgb20b_med;

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
        vtriangles(n) = vtriangle;
        
        %% Verificação da potência de carregamento e descarregamento
        Potcs(n) = Potc;
        Potds(n) = Potd;
        
        %%
        qgb1s(n) = qgb1;
        qgb2s(n) = qgb2;
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

%% Tensão V2L
figure('name','V2L')
plot(Ts,qgb1s,'r-',...
     Ts,qgb2s,'b--','LineWidth',2),zoom
title('Tensão v2L','FontSize',18)
legend('qgb1','qgb2','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")
axis([0 tf -1.1 1.1])

%% Tensões Vg e Vl
figure('name','Tensão Vg') %vg
% plot(Ts,vg_refs,...
%      Ts,vga0a_meds-vha0a_meds+vgb10b_meds-vgb20b_meds,...
%      Ts,vgs),zoom
plot(Ts,vg_refs,'r-',...
     Ts,vga0a_meds-vha0a_meds,'k-',...
     Ts,vgs),zoom
title('Tensão vg','FontSize',18)
legend('vg_{ref}','vg_{med}','vg_{pwm}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")
% axis([0 tf -(Vc3L_ref*2-20) (Vc3L_ref*2+20)])

figure('name','Tensão Vl') %vl
plot(Ts,vl_refs,...
     Ts,vla0a_meds-vha0a_meds,...
     Ts,vls),zoom
title('Tensão vl','FontSize',18)
legend('vl_{ref}','vl_{med}','vl_{pwm}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")
% axis([0 tf -Vc3L_ref-20 Vc3L_ref+20])

%% Correntes
figure('name','Correntes')
plot(Ts,igs,Ts,ils),zoom
title('Correntes','FontSize',18)
legend('ig','il','FontSize',16)
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
% % figure('name','Vg_ref vs modelo')
% % plot(Ts,vg_refs,Ts,-vgb_refs + (vga0a_refs-vha0a_refs),'r--'),zoom
% % title('v*g vs Modelo','FontSize',18)
% % legend('vg_{ref}','vg_{modelo}','FontSize',16)
% % xlabel("Tempo (s)")
% % ylabel("Tensão (V)")
% % axis([0 tf -110 110])
 
% % figure('name','Vl_ref vs modelo')
% % plot(Ts,vl_refs,Ts,vla0a_refs-vha0a_refs,'r--'),zoom
% % title('v*l vs Modelo','FontSize',18)
% % legend('vl_{ref}','vl_{modelo}','FontSize',16)
% % xlabel("Tempo (s)")
% % ylabel("Tensão (V)")
% % axis([0 tf -110 110])
 
% % figure('name','Tensões de polo: 3L')
% % plot(Ts,vga0a_refs,Ts,vha0a_refs,'k-',Ts,vla0a_refs,'b-'),zoom
% % title('Tensões de polo de referência: Conversor 3L','FontSize',18)
% % legend('vga0_{ref}','vha0_{ref}','val0_{ref}','FontSize',16)
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

%% ig vs vg (Ref e Med)
% figure('name','ig vs vg')
% yyaxis left
% plot(Ts,igs,...
%      Ts,ig_refs)
% ylabel("Corrente (A)")
% yyaxis right
% % plot(Ts,vga0a_meds-vha0a_meds+vgb10b_meds-vgb20b_meds,...
% %      Ts,vg_refs)
% plot(Ts,vga0a_meds-vha0a_meds,...
%      Ts,vg_refs)
% title('Corrente e Tensão da rede: medida e de referência','FontSize',18)
% legend('ig_{med}','ig_{ref}','vg_{med}','vg_{ref}','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% % axis([0 tf -Vc3L_ref Vc3L_ref])

%% Tensão no barramento
figure('name','Tensão no barramento 3L')
plot(Ts,Vc3Ls,Ts,Vc3L_refs,'r-'),zoom
title('Tensão do barramento 3L','FontSize',18)
legend('Vdc3L_{med}','Vdc3L_{ref}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")

figure('name','Tensão no barramento 2L')
plot(Ts,Vc2Ls,Ts,Vc2L_refs,'r--',...
    [tsave0 tf],[Vc2L_ref+delta Vc2L_ref+delta],'k-',...
    [tsave0 tf],[Vc2L_ref-delta Vc2L_ref-delta],'k-'),zoom
title('Tensão do barramento 2L ~ Delta = 5%Vdc2L_{ref}','FontSize',18)
legend('Vdc2L_{med}','Vdc2L_{ref}','FontSize',16)
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

wthd_vg = wthdf(vgs, 1/h, f_ref)
wthd_vl = wthdf(vls, 1/h, f_ref)

thd_ig = thdf(igs, 1/h, f_ref)
thd_il = thdf(ils, 1/h, f_ref)

% figure('name','Estado das chaves do conversor 2L')
% plot(Ts,qgb1s,...
%      Ts,qgb2s),zoom
% axis([0 tf -1.1 1.1])

% mean(pots)