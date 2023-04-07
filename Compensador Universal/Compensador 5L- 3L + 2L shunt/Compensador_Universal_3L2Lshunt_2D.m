% Conversor 5L (3L + 2L shunt)

clear;close all;

tic
%% Simulação com parâmetro variado (Sel. parâmetro)
i = 0;
% for Vgm = 440:0.1:460
% for SwSg = 1:0.05:2
% for FPl = 0.1:0.005:1
% for alpha = 0:1*pi/180:100*pi/180
% for FPg = 0.1:0.005:1
% for Vl_ref = 0:1:400
    i = i+1;
%% Condições do Sistema
Vl_ref = 110*sqrt(2);    % Amplitude da tensão de referência braço L

SwSg = 0.95;           % Sobretensão ou afundamento
Vgm = 110*sqrt(2)*SwSg;  % Amplitude da tensão da rede

thetal_ref = 0;          % Fase da tensão de referência braço L

f_ref = 60;              % Frequência de referência
w_ref = 2*pi*f_ref;      % Frequência angular de referência

% Barramento 2L (Conversor B)
Vc2L_ref = 180;          % Tensão de referência do barramento 2L [V]
Vc2L = Vc2L_ref;         % Tensão medida no barramento 2L
Eb = Vc2L_ref;           % Tensão de referência nominal do barramento 2L

% Barramento 3L (Conversor A) 
Vc3L_ref = 180;          % Tensão de referência do barramento 3L [V]
Vc3L = Vc3L_ref;         % Tensão medida no barramento 3L         
Ea = Vc3L_ref;           % Tensão de referência nominal do barramento 3L

Cap = 4.4E-3;            % Capacitância do barramento

%% Impedâncias de entrada e saída
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

% Modelo da rede
Rg = 0.2;                % Resistência
Lg = 7e-3;               % Indutância
Xg = Lg*w_ref;           % Reatância
 
FPg = 1;                 % Fator de potência da rede
thetag = acos(FPg);      % Defasagem tensão-corrente da rede

%% Parâmetros de Simulação
h = 1E-7;                % Passo de cálculo
t = 0;                   % Tempo inicial de simulação
tf = 2;                  % Tempo final de simulação

%% Parâmetros de Gravação
% tsave0 = tf-1/60;             % Tempo inicial de gravação
tsave0 = 0;
tsave = tsave0;          % Tempo de gravação
npt = 100000;            % Dimensão do vetor de saída de dados

hsave = (tf-tsave0)/npt; % Passo de de gravação dos vetores de saída de dados
hsave = 1E-5;

if hsave < h             % Sendo o Passo de gravação menor que o passo de cálculo (Saturação)
    hsave = h;           % Defina o passo de gravação = passo de cálculo
    npt = (tf-tsave0)/hsave; % Recalcule o a dimensão dos vetores de saída de dados
end

n = 0;                   % Inicialização da variável de posição dos vetores de saída

%% Inicialização das variáveis
ig = 0;                  % Corrente inicial no ramo g
il = 0;                  % Corrente inicial no ramo l
ih = 0;                  % Corrente inicial no ramo s

% 3L
vha0_int = 0;            % valor inicial da integral da tensão no polo ha
vga0_int = 0;            % valor inicial da integral da tensão no polo ga
vla0_int = 0;            % valor inicial da integral da tensão no polo la

%2L
vhb10_int = 0;           % valor inicial da integral da tensão no polo hb1
vhb20_int = 0;           % valor inicial da integral da tensão no polo hb2

% Carga
el_int = 0;              % Valor inicial da integral da tensão na carga

control = -1;
cont1 = 0;
cont2 = 0;

k=0;                     % Índice de salvamento da potência

%% Parâmetros do controlador 
% Controlador PI
Kp = 0.2;
Ki = 15;

P_error = 0;
I_error = 0;

vc_ref = Vc3L;
vc3L = Vc3L;

Ig = 0;
Ig0 = 3;

% Controlador por Histerese
delta = Vc2L_ref*0.02;   % HistereseIgc/sqrt(2)x
delta_V2L = Vc2L_ref*0.02;

%% Cálculo da Amplitude da corrente drenada pela rede
fasegl = cos(thetal_ref - acos(FPl));
Igc = min(roots([Rg/2, -(Vgm/2 + Rg*Il*fasegl), Pl+(Rg*Il^2)/2]));

%% Amplitude e Fase da tensão gerada pelo conversor shunt

Is     = sqrt(Igc^2 + Il^2 - 2*Igc*Il * fasegl);
faseIs = acos((Igc^2 + Is^2 - Il^2)/(2*Igc*Is));
Vsh    = Vgm - (Rg + 1i*Xg)*Is*(cos(faseIs)+1i*sin(faseIs));

Vshref = abs(Vsh);
fasesh = angle(Vsh);

%% Inicialização da onda triangular (Normalizada)
ttri = 0;           % Tempo inicial da onda triangular
ftri = 10E3;        % Frequência da onda triangular de 10 kHz
htri = 1/ftri;      % Período da onda triangular
vtri = 1/2;         % Tensão máxima da onda triangular
dtri = 1/htri;      % Derivada da onda triangular (dV/dT)
sign = -1;          % Sinal inicial da derivada (Comportamento decrescente)
atrasoPWM = 0.5*2.*pi*f_ref/ftri; % Atraso gerado no PWM

sobretensao = 0;
%--------------------------------------------------------------------
%% Início do Looping
while t<tf
    t = t + h;
    
%     if t >= tf/2
%         Vgm = 300*sqrt(2);
%     end
    
    eg = Vgm*cos(w_ref*t);
    
    %% Onda triangular
    if t >= ttri
        ttri = ttri + htri;
        
        %% Controlador PI da corrente ig
        vc_error = Vc2L_ref - Vc2L;
        P_error = Kp*vc_error;
        I_error = I_error + htri*Ki*vc_error;
        Ig = P_error + I_error + Ig0;
        
        ig_ref = Ig*cos(w_ref*t - thetag);
        ig_error = (ig_ref - ig);
        
        ig_ref = Igc*cos(w_ref*t);
        il_ref = Il*cos(w_ref*t - acos(FPl));
        ih_ref = ig_ref-il_ref;
        
        %% Tensões de referência
%         vg_ref = eg - Rg*ig - (Lg/htri)*ig_error;
        vg_ref = Vshref*cos(w_ref*t + atrasoPWM + fasesh);
        vl_ref = eg - Vl_ref*cos(w_ref*t + thetal_ref + atrasoPWM);        
        vel_ref = Vl_ref*cos(w_ref*t + thetal_ref + atrasoPWM); 
        
        if sobretensao == 1 % EVENTO DE SOBRETENSÃO
            %% Fatores de repartição
            % Vx
            vxmaxhab =  (Vc2L_ref+Vc3L_ref/2) + max([vg_ref,0]);
            vxmingba = -(Vc2L_ref+Vc3L_ref/2) + min([vg_ref,0]);

            vxmaxla0  =  Vc3L_ref/2 + max([vl_ref,0]);
            vxminla0  = -Vc3L_ref/2 + min([vl_ref,0]);

            vxmax = min([vxmaxhab,vxmaxla0]);
            vxmin = max([vxmingba,vxminla0]);

            vx = (vxmax+vxmin)/2;

    %         if vg_ref >= 0
    %             vx = vxmax;
    %         else
    %             vx = vxmin;
    %         end

            % Vy
            vhab_ref  = vx - vg_ref;

    %         vymaxha0 =  Vc3L_ref/2 - max(vhab_ref/2,0);
    %         vyminha0 = -Vc3L_ref/2 - min(vhab_ref/2,0);
    %         
    %         vymaxhb  =  Vc2L_ref + max(vhab_ref/2,0);
    %         vyminhb  = -Vc2L_ref + min(vhab_ref/2,0);

    %         vymaxha0 =  Vc3L_ref/2 - max(vhab_ref/2,-vhab_ref/2);
    %         vyminha0 = -Vc3L_ref/2 - min(vhab_ref/2,-vhab_ref/2);
    %         
    %         vymaxhb  =  Vc2L_ref + max(vhab_ref/2,-vhab_ref/2);
    %         vyminhb  = -Vc2L_ref + min(vhab_ref/2,-vhab_ref/2);

            vymaxha0 =  Vc3L_ref/2 - vhab_ref/2;
            vyminha0 = -Vc3L_ref/2 - vhab_ref/2;

            vymaxhb  =  Vc2L_ref + vhab_ref/2;
            vyminhb  = -Vc2L_ref + vhab_ref/2;

            vymax = min([vymaxha0,vymaxhb]);
            vymin = max([vyminha0,vyminhb]);

    %         vy = (vymax+vymin)/2;

            % Controle do barramento 3L por histerese - SOBRETENSÃO
            Vc3L_error = Vc3L_ref - Vc3L;

            if Vc3L_error >= delta % Vc3L_ref > Vc3L -> Carregar
                control = 1;
            elseif Vc3L_error <= -delta %  Descarregar
                control = -1;
            end

    %         control = -1;

            if     control == 1 % Carregar
                cont1 = cont1+1;
                if ih >= 0
                    vy = vymin;
                else
                    vy = vymax;
                end

            elseif control == -1   % Descarregar
                cont2 = cont2+1;
                if ih >= 0
                    vy = vymax;
                else
                    vy = vymin;
                end
            end

            vhb_ref = -vhab_ref/2 + vy;

            %% Tensões de polo de referência
            % 3L
            vga0_ref = vx;
            vha0_ref = vhab_ref/2 + vy;
            vla0_ref = vx - vl_ref;        

            % 2L
            vhb10_ref = vhb_ref/2;
            vhb20_ref = -vhb_ref/2;
        else
            %% Cálculo das tensões de polo - StandBy
            Vs = [vg_ref,vl_ref,0];
            vxmax =  Vc3L_ref/2 + max(Vs);
            vxmin = -Vc3L_ref/2 + min(Vs);
            vx = (vxmax+vxmin)/2;
            
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
                        vhb10_ref = v2L_ref/2;
                        vhb20_ref = -v2L_ref/2;
                    else
                        v2L_ref = -delta_V2L;
                        vhb10_ref = v2L_ref/2;
                        vhb20_ref = -v2L_ref/2;
                    end
                else
                    v2L_ref = 0;
                    vhb10_ref = 1.05*Vc2L_ref;
                    vhb20_ref = 1.05*Vc2L_ref;
                    control = 0;
                end

            elseif control == -1   % Descarregar          
                cont2 = cont2 + 1;
                if Vc2L > Vc2L_ref
                    if ig >= 0
                        v2L_ref = -delta_V2L;
                        vhb10_ref = v2L_ref/2;
                        vhb20_ref = -v2L_ref/2;
                    else
                        v2L_ref = delta_V2L;
                        vhb10_ref = v2L_ref/2;
                        vhb20_ref = -v2L_ref/2;
                    end
                else
                    v2L_ref = 0;
                    vhb10_ref = 1.05*Vc2L_ref;
                    vhb20_ref = 1.05*Vc2L_ref;
                    control = 0;
                end
            end
            
            vga0_ref = vx;
            vha0_ref = vx - vg_ref - v2L_ref;
            vla0_ref = vx - vl_ref;
            
        end
        %% Tensões médias nos braços
        % 3L
        vha0_med = vha0_int/htri;
        vga0_med = vga0_int/htri;
        vla0_med = vla0_int/htri;
        
        vha0_int = 0;
        vga0_int = 0;
        vla0_int = 0;
        
        % 2L
        vhb10_med = vhb10_int/htri;
        vhb20_med = vhb20_int/htri;
        
        vhb10_int = 0;
        vhb20_int = 0;
        
        % Carga
        el_med = el_int/htri;        
        el_int = 0;
        
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
    vtri = vtri + sign*dtri*h;
    vt3L = vtri*Vc3L;
    vt2L = vtri*Vc2L;
    
    %% Estado das chaves
    % 3L
    if vha0_ref >= vt3L % Estado das chaves (Braço h)
        qha = 1; % Chave fechada
    else
        qha = 0; % Chave aberta
    end
    
    if vga0_ref >= vt3L % Estado das chaves (Braço g)
        qga = 1; % Chave fechada
    else
        qga = 0; % Chave aberta
    end
    
    if vla0_ref >= vt3L % Estado das chaves (Braço l)
        qla = 1; % Chave fechada (IGBT em condução)
    else
        qla = 0; % Chave aberta (IGBT em corte)
    end
    
    % 2L
    if vhb10_ref >= vt2L % Estado das chaves (Braço l)
        qhb1 = 1; % Chave fechada (IGBT em condução)
    else
        qhb1 = 0; % Chave aberta (IGBT em corte)
    end
    
    if vhb20_ref >= vt2L % Estado das chaves (Braço l)
        qhb2 = 1; % Chave fechada (IGBT em condução)
    else
        qhb2 = 0; % Chave aberta (IGBT em corte)
    end
    
    %% Tensões de polo
    % 3L
    vha0 = (2*qha - 1)*(Vc3L/2);
    vga0 = (2*qga - 1)*(Vc3L/2);
    vla0 = (2*qla - 1)*(Vc3L/2);
    
    % 2L
    vhb10 = (2*qhb1 - 1)*(Vc2L/2);
    vhb20 = (2*qhb2 - 1)*(Vc2L/2);
    
    %% Tensões geradas
    vg = vga0 - vha0 + vhb10 - vhb20;
    vl = vga0 - vla0;
    el = eg - vl;
    
    %% Integração das tensões de polo
    % 3L
    vha0_int = vha0_int + vha0*h;
    vga0_int = vga0_int + vga0*h;
    vla0_int = vla0_int + vla0*h;  
    
    % 2L
    vhb10_int = vhb10_int + vhb10*h;
    vhb20_int = vhb20_int + vhb20*h;
    
    % Carga
    el_int = el_int + el*h;
    
    %% Integração numérica das correntes ig e il
    il = il*(1 - h*Rl/Ll) + (h/Ll)*(eg - vl);
    ih = ih*(1 - h*Rg/Lg) + (h/Lg)*(eg - vg);
    ig = ih + il;
    
    %% Controle do barramento
    % 3L
    Rp3L = 1e12;
    Rs3L = 0;
%     ik3L = ig*qga - il*qla - ih*qha;
    ik3L = ig_ref*qga - il_ref*qla - ih_ref*qha;
    ic3L = (Rp3L*ik3L - Vc3L)/(Rp3L + Rs3L);   % Corrente de entrada barramento 2L
    Vc3L = Vc3L + (h/Cap)*ic3L;                % Tensão barramento 3L
    Vc3L = Vc3L_ref*1.00;
    
    % 2L
    Rp2L = 1e12;
    Rs2L = 0.0;
%     ik2L = ih*qhb1 - ih*qhb2;
    ik2L = ih_ref*qhb1 - ih_ref*qhb2;
    ic2L = (Rp2L*ik2L - Vc2L)/(Rp2L + Rs2L);   % Corrente de entrada barramento 2L
    Vc2L = Vc2L + (h/Cap)*ic2L;                % Tensão barramento 2L
    Vc2L = Vc2L_ref*1.00;
    

    %% Potência de carregamento e descarregamento
%     if     control == 1  % Carregar 3L
%         Potc = ic3L*Vc3L;
%         Potd = 0;
%     elseif control == -1 % Descarregar 3L
%         Potd = ic3L*Vc3L;
%         Potc = 0;
%     else
%         err0 = 1
%     end
%     
%     % Salvamento da potência de carregamento e descarregamento
%     k = k+1;
%     Potcs(k) = Potc;
%     Potds(k) = Potd;
    
    %% Salvamento das variáveis
    if tsave <= t
        tsave = tsave + hsave;
        n = n + 1;
        
        % Tempo da simulação
        Ts(n) = t;
        
        % Estado das chaves
        qhs(n) = qha;
        qgs(n) = qga;
        qls(n) = qla;
        
        qhb1s(n) = qhb1;
        qhb2s(n) = qhb2;
        
        % Tensões de polo de referência
        vh0_refs(n) = vha0_ref;
        vg0_refs(n) = vga0_ref;
        vl0_refs(n) = vla0_ref;
        
        vhb10_refs(n) = vhb10_ref;
        vhb20_refs(n) = vhb20_ref;
        
        % Tensões de referência 
        vg_refs(n) = vg_ref;
        vl_refs(n) = vl_ref;  
        vel_refs(n)= vel_ref; 
        
        % Tensões de polo
        vh0s(n) = vha0;
        vg0s(n) = vga0;
        vl0s(n) = vla0;
        
        % Tensões geradas
        vgs(n) = vg;
        vls(n) = vl;    
        els(n) = el; % Carga
        
        % Tensões médias
        vh0_meds(n) = vha0_med;
        vg0_meds(n) = vga0_med;
        vl0_meds(n) = vla0_med;   
        el_meds(n) = el_med; % Carga
        
        vhb10_meds(n) = vhb10_med; 
        vhb20_meds(n) = vhb20_med;
        
        % Corrente de referência do controlador
        ig_refs(n) = ig_ref;
        
        % Correntes
        ihs(n) = ih;
        igs(n) = ig;
        ils(n) = il;
        
        % Barramento
        Vc3Ls(n) = Vc3L;
        Vc3L_refs(n) = Vc3L_ref;
        Vc2Ls(n) = Vc2L;
        Vc2L_refs(n) = Vc2L_ref;
        
        % Portadora triangular
        vtris(n) = vtri;
        vt3Ls(n) = vt3L;
        vt2Ls(n) = vt2L;
        
        
        % Correntes de referência
        ig_refs(n)=ig_ref;
        il_refs(n)=il_ref;
        ih_refs(n)=ih_ref;
    end
end
% mean(Potcs)
% mean(Potds)
% Vgms(i) = Vgm;
% FPls(i) = FPl;
% alphas(i) = alpha;
% FPgs(i) = FPg;
% Vls(i) = Vl_ref;

% P2s(i) = mean(Potds);
% P2cs(i) = mean(Potcs);
% SwSgs(i) = SwSg;

% end
toc

% figure('name','Potência de descarregamento')
% plot(Vgms,P2s,'r-',[440 460],[0 0],'k-')
% plot(FPls,P2s,'r-',[0.1 1],[0 0],'k-','LineWidth',2)
% plot(alphas.*180/pi,P2s,'r-',[0 100],[0 0],'k-','LineWidth',2)
% plot(FPgs,P2s,'r-',[0.1 1],[0 0],'k-','LineWidth',2)
% plot(Vls,P2s,'r-',[0 400],[0 0],'k-')

% plot(SwSgs,P2s,'r-',[1 2],[0 0],'k-')
% plot(SwSgs,P2cs,'r-',[1 2],[0 0],'k-')

% title('Potência de Descarregamento','FontSize',18)
% title('Potência de Carregamento','FontSize',18)

% xlabel("Tensão da rede (Vpico)")
% xlabel("Fator de potência da carga")
% xlabel("Diferença de fase entre Vg e Vl (º)")
% xlabel("Fator de potência do grid")
% xlabel("Tensão da carga (Vpico)")
% xlabel("Sobretensão (p.u.)")

% ylabel("Potência Média")
% grid minor;

%% Plots

% ---- Tensões Vg e Vl
figure('name','Tensão Vg') %vg
plot(Ts,vg_refs,...
     Ts,vg0_meds-vh0_meds+vhb10_meds-vhb20_meds,...
     Ts,vgs),zoom
title('Tensão vg','FontSize',18)
legend('vg_{ref}','vg_{med}','vg_{pwm}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")
% axis([0 tf -(Vc3L_ref*2-20) (Vc3L_ref*2+20)])

figure('name','Tensão Vl') %vl
plot(Ts,vl_refs,...
     Ts,vg0_meds-vl0_meds,...
     Ts,vls),zoom
title('Tensão vl','FontSize',18)
legend('vl_{ref}','vl_{med}','vl_{pwm}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")
% axis([0 tf -Vc3L_ref-20 Vc3L_ref+20])

% ---- Tensão na carga
figure('name','Tensão Vel') %vg
plot(Ts,vel_refs,...
     Ts,el_meds,...
     Ts,els),zoom
title('Tensão vel','FontSize',18)
legend('vel_{ref}','vel_{med}','vel_{pwm}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")
% axis([0 tf -(Vc3L_ref*2-20) (Vc3L_ref*2+20)])

% ---- Correntes
figure('Name','Corrente do circuito: Lado G, A e L sem afundamento')
plot(Ts,igs,Ts,ils,Ts,ihs,'r-','LineWidth',1),zoom
title('Corrente do circuito: ig, il e ia')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
legend('ig','il','ih','FontSize',18)
grid()

% ---- Correntes
figure('Name','Corrente do circuito: Lado G, A e L sem afundamento')
plot(Ts,ig_refs,Ts,il_refs,Ts,ih_refs,'r-','LineWidth',1),zoom
title('Corrente do circuito: ig, il e ia')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
legend('ig','il','ih','FontSize',18)
grid()

% ---- Tensão barramento
figure('name','Tensão no barramento 2L')
plot(Ts,Vc2Ls,Ts,Vc2L_refs,'r--'),zoom
title('Tensão do barramento 2L','FontSize',18)
legend('Vdc2L_{med}','Vdc2L_{ref}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid('minor')

figure('name','Tensão no barramento 3L')
plot(Ts,Vc3Ls,Ts,Vc3L_refs,'r--',...
    [tsave0 tf],[Vc3L_ref+delta Vc3L_ref+delta],'k-',...
    [tsave0 tf],[Vc3L_ref-delta Vc3L_ref-delta],'k-'),zoom
title('Tensão do barramento 3L','FontSize',18)
legend('Vdc3L_{med}','Vdc3L_{ref}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid('minor')