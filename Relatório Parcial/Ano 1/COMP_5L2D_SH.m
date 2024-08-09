% Conversor 5L (3L + 2L shunt)

clear;clc;close all;
tic


%% Simulação com parâmetro variado (Sel. parâmetro)
i = 0;
% for Vgm = 440:0.1:460
% for SwSg = 1:0.05:2
% for FPl = 0.1:0.005:1
% for alpha = 0:1*pi/180:100*pi/180
% for FPg = 0.1:0.005:1
% for Vel_ref = 0:1:400
    i = i+1;

    
%% Condições do Sistema
SwSg = 0.95;             % Sobretensão ou afundamento
Vgm = 110*sqrt(2)*SwSg;  % Amplitude da tensão da rede
Vel_ref = 110*sqrt(2);   % Amplitude da tensão de referência sobre a carga

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
% Cfl = 120E-9;


%% Impedâncias de entrada e saída

%   CARGA
% Dados da carga
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

%   SHUNT
% Dados do Shunt
Rs = 0.2;                % Resistência
Ls = 7e-3;               % Indutância
Xs = Ls*w_ref;           % Reatância
 
%   REDE
% Fator de potência
FPg = 1;                 % Fator de potência da rede
thetag = acos(FPg);      % Defasagem tensão-corrente da rede


%% Parâmetros de Simulação
h = 1E-7;                % Passo de cálculo
t = 0;                   % Tempo inicial de simulação
tf = 3;                  % Tempo final de simulação


%% Parâmetros de Gravação
tsave0 = 0;
tsave = tsave0;          % Tempo de gravação
npt = 20000;            % Dimensão do vetor de saída de dados

hsave = (tf-tsave0)/npt; % Passo de de gravação dos vetores de saída de dados
hsave = 1E-5;

if hsave < h             % Sendo o Passo de gravação menor que o passo de cálculo (Saturação)
    hsave = h;           % Defina o passo de gravação = passo de cálculo
    npt = (tf-tsave0)/hsave; % Recalcule o a dimensão dos vetores de saída de dados
end

n = 0;                   % Inicialização da variável de posição dos vetores de saída

%% Inicialização das variáveis
vsh = 0;
ig = 0;                  % Corrente inicial no ramo g
il = 0;                  % Corrente inicial no ramo l
is = 0;                  % Corrente inicial no ramo s

% 3L
vsa0_int = 0;            % valor inicial da integral da tensão no polo sa
vga0_int = 0;            % valor inicial da integral da tensão no polo ga
vla0_int = 0;            % valor inicial da integral da tensão no polo la

%2L
vsb10_int = 0;           % valor inicial da integral da tensão no polo sb1
vsb20_int = 0;           % valor inicial da integral da tensão no polo sb2

% Carga
el_int = 0;              % Valor inicial da integral da tensão na carga

control = 1;
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
delta = Vc2L_ref*0.02;
delta_V2L = Vc2L_ref*0.05; % Tensão total de referência máxima do conversor 2L 

%% Cálculo de Regime Permanente
    %% Cálculo da Amplitude da corrente drenada pela rede
    fasegl = cos(thetal_ref - acos(FPl));
    Igc = min(roots([Rs/2, -(Vgm/2 + Rs*Il*fasegl), Pl+(Rs*Il^2)/2]));

    
    %% Amplitude  e Fase da tensão gerada na compensação shunt
    Is     = sqrt(Igc^2 + Il^2 - 2*Igc*Il * fasegl);
    faseIs = acos((Igc^2 + Is^2 - Il^2)/(2*Igc*Is));
    Vsh    = Vgm - (Rs + 1i*Xs)*Is*(cos(faseIs)+1i*sin(faseIs));

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


%% Sobretensão
sobretensao = 0;

%--------------------------------------------------------------------
%% Início do Looping
while t<tf
    t = t + h;
    
    if t > tf/2
        sobretensao = 1;
    end 

% SOBRETENSÃO
    if sobretensao == 1
        Vgm = 1.5*110*sqrt(2);
        Vc3L_ref = Ea*1.67;
    else
        Vgm = 1.00*110*sqrt(2); 
    end
    
    eg = Vgm*cos(w_ref*t);    % Tensão da rede elétrica
    
    %% Onda triangular
    if t >= ttri
        ttri = ttri + htri;    % Atualização da onda triangular
        
        %% Controlador PI da corrente ig
        if sobretensao == 0 % CONDIÇÕES NOMINAIS
            Vc3L_error = Vc3L_ref - Vc3L;            % Erro controle barramento
            P_error = Kp*Vc3L_error;                 % Erro proporcional
            I_error = I_error + htri*Ki*Vc3L_error;  % Erro integral
            Ig = P_error + I_error + Ig0;            % Amplitude ig*

            ig_ref = Ig*cos(w_ref*t - thetag);       % ig*
            ig_error = (ig_ref - ig);                % Erro ig
            
        else                % EVENTO DE SOBRETENSÃO
            Vc2L_error = Vc2L_ref - Vc2L;            % Erro controle barramento
            P_error = Kp*Vc2L_error;                 % Erro proporcional
            I_error = I_error + htri*Ki*Vc2L_error;  % Erro integral
            Ig = P_error + I_error + Ig0;            % Amplitude ig*

            ig_ref = Ig*cos(w_ref*t - thetag);       % ig*
            ig_error = (ig_ref - ig);
            
        end
        %% Correntes de Regime Permanente
%         ig_ref = Igc*cos(w_ref*t);             
        il_ref = Il*cos(w_ref*t - thetal);
        is_ref = ig_ref - il_ref;
        is_error = (is_ref  - is);
        
        %% Tensões de referência
        vsh_ref = eg - Rs*is - (Ls/htri)*is_error;
%         vsh_ref = Vshref*cos(w_ref*t + atrasoPWM + fasesh);
        vel_ref = Vel_ref*cos(w_ref*t + thetal_ref + atrasoPWM);        
        vse_ref = eg - vel_ref;

%         vse_ref = eg - vel_ref;
        
        vsh_ref_error = vsh_ref - vsh;
        %% Condição da rede
        if sobretensao == 1 % EVENTO DE SOBRETENSÃO
            %% Fatores de repartição
            % Vx
            vxmaxsab =  (Vc2L_ref+Vc3L_ref/2) + max([vsh_ref,0]);
            vxminsab = -(Vc2L_ref+Vc3L_ref/2) + min([vsh_ref,0]);

            vxmaxla0  =  Vc3L_ref/2 + max([vse_ref,0]);
            vxminla0  = -Vc3L_ref/2 + min([vse_ref,0]);

            vxmax = min([vxmaxsab,vxmaxla0]);
            vxmin = max([vxminsab,vxminla0]);

            vx = (vxmax+vxmin)/2;

    %         if vsh_ref >= 0
    %             vx = vxmax;
    %         else
    %             vx = vxmin;
    %         end

            % Vy
            vsab_ref  = vx - vsh_ref;

            vymaxsa0 =  Vc3L_ref/2 - vsab_ref/2;
            vyminsa0 = -Vc3L_ref/2 - vsab_ref/2;

            vymaxsb  =  Vc2L_ref + vsab_ref/2;
            vyminsb  = -Vc2L_ref + vsab_ref/2;

            vymax = min([vymaxsa0,vymaxsb]);
            vymin = max([vyminsa0,vyminsb]);

    %         vy = (vymax+vymin)/2;

            % Controle do barramento 3L por histerese - SOBRETENSÃO
            Vc3L_error = Vc3L_ref - Vc3L;

            if Vc3L_error >= delta % Vc3L_ref > Vc3L -> Carregar
                control = 1;
            elseif Vc3L_error <= -delta %  Descarregar
                control = -1;
            end

    %         control = -1;
            vy = (vymin+vymax)/2;
            if     control == 1 % Carregar
                cont1 = cont1+1;
                if is >= 0
                    vy = vymin;
                else
                    vy = vymax;
                end

            elseif control == -1   % Descarregar
                cont2 = cont2+1;
                if is >= 0
                    vy = vymax;
                else
                    vy = vymin;
                end
            end

            vsb_ref = -vsab_ref/2 + vy;

            %% Tensões de polo de referência
            % 3L
            vga0_ref = vx;
            vsa0_ref = vsab_ref/2 + vy;
            vla0_ref = vx - vse_ref;        

            % 2L
            vsb10_ref = vsb_ref/2;
            vsb20_ref = -vsb_ref/2;
        else    % CONDIÇÕES NOMINAIS
            
            %% Cálculo das tensões de polo - StandBy
            Vs = [vsh_ref,vse_ref,0];
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
                    if is >= 0
                        v2L_ref = delta_V2L;
                        vsb10_ref = v2L_ref/2;
                        vsb20_ref = -v2L_ref/2;
                    else
                        v2L_ref = -delta_V2L;
                        vsb10_ref = v2L_ref/2;
                        vsb20_ref = -v2L_ref/2;
                    end
                else
                    v2L_ref = 0;
                    vsb10_ref = 1.05*Vc2L_ref;
                    vsb20_ref = 1.05*Vc2L_ref;
                    control = 0;
                end

            elseif control == -1   % Descarregar          
                cont2 = cont2 + 1;
                if Vc2L > Vc2L_ref
                    if is >= 0
                        v2L_ref = -delta_V2L;
                        vsb10_ref = v2L_ref/2;
                        vsb20_ref = -v2L_ref/2;
                    else
                        v2L_ref = delta_V2L;
                        vsb10_ref = v2L_ref/2;
                        vsb20_ref = -v2L_ref/2;
                    end
                else
                    v2L_ref = 0;
                    vsb10_ref = 1.05*Vc2L_ref;
                    vsb20_ref = 1.05*Vc2L_ref;
                    control = 0;
                end
            end
            
            vga0_ref = vx;
            vsa0_ref = vx - vsh_ref + v2L_ref;
            vla0_ref = vx - vse_ref;
            
            
        end
        %% Tensões médias nos braços
        % 3L
        vsa0_med = vsa0_int/htri;
        vga0_med = vga0_int/htri;
        vla0_med = vla0_int/htri;
        
        vsa0_int = 0;
        vga0_int = 0;
        vla0_int = 0;
        
        % 2L
        vsb10_med = vsb10_int/htri;
        vsb20_med = vsb20_int/htri;
        
        vsb10_int = 0;
        vsb20_int = 0;
        
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
    if vsa0_ref >= vt3L % Estado das chaves (Braço s)
        qsa = 1; % Chave fechada
    else
        qsa = 0; % Chave aberta
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
    if vsb10_ref >= vt2L % Estado das chaves (Braço l)
        qsb1 = 1; % Chave fechada (IGBT em condução)
    else
        qsb1 = 0; % Chave aberta (IGBT em corte)
    end
    
    if vsb20_ref >= vt2L % Estado das chaves (Braço l)
        qsb2 = 1; % Chave fechada (IGBT em condução)
    else
        qsb2 = 0; % Chave aberta (IGBT em corte)
    end
    
    %% Tensões de polo
    % 3L
    vsa0 = (2*qsa - 1)*(Vc3L/2);
    vga0 = (2*qga - 1)*(Vc3L/2);
    vla0 = (2*qla - 1)*(Vc3L/2);
    
    % 2L
    vsb10 = (2*qsb1 - 1)*(Vc2L/2);
    vsb20 = (2*qsb2 - 1)*(Vc2L/2);
    
    %% Tensões geradas
    vsh = vga0 - vsa0 + vsb10 - vsb20;
    vse = vga0 - vla0;
    el = eg - vse;
    
    %% Integração das tensões de polo
    % 3L
    vsa0_int = vsa0_int + vsa0*h;
    vga0_int = vga0_int + vga0*h;
    vla0_int = vla0_int + vla0*h;  
    
    % 2L
    vsb10_int = vsb10_int + vsb10*h;
    vsb20_int = vsb20_int + vsb20*h;
    
    % Carga
    el_int = el_int + el*h;
    
    %% Integração numérica das correntes ig e il
    il = il*(1 - h*Rl/Ll) + (h/Ll)*(eg - vse);
    is = is*(1 - h*Rs/Ls) + (h/Ls)*(eg - vsh);
    ig = is + il;
    
    %% Controle do barramento
    % 3L
    Rp3L = 5e3;
    Rs3L = 0.250;
    ik3L = ig*qga - il*qla - is*qsa;
%     ik3L = ig_ref*qga - il_ref*qla - is_ref*qsa;
    ic3L = (Rp3L*ik3L - Vc3L)/(Rp3L + Rs3L);   % Corrente de entrada barramento 2L
    Vc3L = Vc3L + (h/Cap)*ic3L;                % Tensão barramento 3L
%     Vc3L = Vc3L_ref*1.00;
    
    % 2L
    Rp2L = 5e3;
    Rs2L = 0.250;
    ik2L = is*qsb1 - is*qsb2;
%     ik2L = is_ref*qsb1 - is_ref*qsb2;
    ic2L = (Rp2L*ik2L - Vc2L)/(Rp2L + Rs2L);   % Corrente de entrada barramento 2L
    Vc2L = Vc2L + (h/Cap)*ic2L;                % Tensão barramento 2L
%     Vc2L = Vc2L_ref*1.00;
    

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
    
    % Salvamento da potência de carregamento e descarregamento
%     k = k+1;
%     Potcs(k) = Potc;
%     Potds(k) = Potd;
    
    %% Salvamento das variáveis
    if tsave <= t
        tsave = tsave + hsave;
        n = n + 1;
        
        % Tempo da simulação
        Ts(n) = t;
        
        % Rede
        egs(n) = eg;
        
        % Estado das chaves
        qss(n) = qsa;
        qgs(n) = qga;
        qls(n) = qla;
        
        qsb1s(n) = qsb1;
        qsb2s(n) = qsb2;
        
        % Tensões de polo de referência
        vs0_refs(n) = vsa0_ref;
        vg0_refs(n) = vga0_ref;
        vl0_refs(n) = vla0_ref;
        
        vsb10_refs(n) = vsb10_ref;
        vsb20_refs(n) = vsb20_ref;
        
        % Tensões de referência 
        vsh_refs(n) = vsh_ref;
        vse_refs(n) = vse_ref;  
        vel_refs(n)= vel_ref; 
        
        % Tensões de polo
        vs0s(n) = vsa0;
        vg0s(n) = vga0;
        vl0s(n) = vla0;
        
        % Tensões geradas
        vshs(n) = vsh;
        vses(n) = vse;    
        els(n) = el; % Carga
        
        % Tensões médias
        vs0_meds(n) = vsa0_med;
        vg0_meds(n) = vga0_med;
        vl0_meds(n) = vla0_med;   
        el_meds(n) = el_med; % Carga
        
        vsb10_meds(n) = vsb10_med; 
        vsb20_meds(n) = vsb20_med;
        
        % Corrente de referência do controlador
        ig_refs(n) = ig_ref;
        
        % Correntes
        iss(n) = is;
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
        is_refs(n)=is_ref;
        
        % Erros
        ig_errors(n)=ig_error;
        vsh_ref_errors(n)=vsh_ref_error;
        
    end
end
% mean(Potcs)
% mean(Potds)
% Vgms(i) = Vgm;
% FPls(i) = FPl;
% alphas(i) = alpha;
% FPgs(i) = FPg;
% Vels(i) = Vel_ref;

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
% plot(Vels,P2s,'r-',[0 400],[0 0],'k-')

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
% figure('name','Tensão Vg') %vsh
% plot(Ts,vsh_refs,...
%      Ts,vg0_meds-vs0_meds+vsb10_meds-vsb20_meds,...
%      Ts,vshs),zoom
% title('Tensão vsh','FontSize',18)
% legend('vg_{ref}','vg_{med}','vg_{pwm}','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% % axis([0 tf -(Vc3L_ref*2-20) (Vc3L_ref*2+20)])
% 
% figure('name','Tensão Vl') %vse
% plot(Ts,vse_refs,...
%      Ts,vg0_meds-vl0_meds,...
%      Ts,vses),zoom
% title('Tensão vse','FontSize',18)
% legend('vl_{ref}','vl_{med}','vl_{pwm}','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% axis([0 tf -Vc3L_ref-20 Vc3L_ref+20])

% ---- Tensão na carga
% figure('name','Tensão Vel') %vsh
% plot(Ts,vel_refs,...
%      Ts,el_meds,...
%      Ts,els),zoom
% title('Tensão vel','FontSize',18)
% legend('vel_{ref}','vel_{med}','vel_{pwm}','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% axis([0 tf -(Vc3L_ref*2-20) (Vc3L_ref*2+20)])

% ---- Correntes
% figure('Name','Corrente do circuito: Lado G, S e L')
% plot(Ts,igs,Ts,ils,Ts,iss,'r-','LineWidth',1),zoom
% title('Corrente do circuito: ig, il e ia')
% xlabel("Tempo (s)")
% ylabel("Corrente (A)")
% legend('ig','il','is','FontSize',18)
% grid()

% ---- Correntes
% figure('Name','Corrente de Referência do circuito: Lado G, S e L')
% plot(Ts,ig_refs,Ts,il_refs,Ts,is_refs,'r-','LineWidth',1),zoom
% title('Corrente de Referência do circuito: ig, il e ia')
% xlabel("Tempo (s)")
% ylabel("Corrente (A)")
% legend('ig','il','is','FontSize',18)
% grid()

% ---- Tensão barramento
% figure('name','Tensão no barramento 2L')
% plot(Ts,Vc2Ls,Ts,Vc2L_refs,'r--',...
%     [tsave0 tf],[Vc2L_ref+delta Vc2L_ref+delta],'k-',...
%     [tsave0 tf],[Vc2L_ref-delta Vc2L_ref-delta],'k-'),zoom
% title('Tensão do barramento 2L','FontSize',18)
% legend('Vdc2L_{med}','Vdc2L_{ref}','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% grid('minor')
% 
% figure('name','Tensão no barramento 3L')
% plot(Ts,Vc3Ls,Ts,Vc3L_refs,'r--',...
%     [tsave0 tf],[Vc3L_ref+delta Vc3L_ref+delta],'k-',...
%     [tsave0 tf],[Vc3L_ref-delta Vc3L_ref-delta],'k-'),zoom
% title('Tensão do barramento 3L','FontSize',18)
% legend('Vdc3L_{med}','Vdc3L_{ref}','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% grid('minor')

% % ---- Erros
% figure('name','Erros das correntes de referência')
% plot(Ts,ig_errors,Ts,vsh_ref_errors,'r-'),zoom
% title('Erro corrente de referência','FontSize',18)
% legend('ig_{error}','vsh_{error}','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% grid('minor')

% ---- Estados da chaves
% figure('name','Tesnão V2L')
% plot(Ts,vsb10_meds-vsb20_meds,'r-'),zoom
% title('Estado das chaves 2L','FontSize',18)
% legend('Chave qsb1','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% grid('minor')



%% Figuras do Relatório Parcial
% Barramentos

ax1 = [0 3 150 320];
sp1 = subplot(3,1,1);
plot(Ts,Vc3Ls,Ts,Vc3L_refs,'r--',...
    [tf/2 tf],[Vc3L_ref+delta Vc3L_ref+delta],'k-',...
    [tf/2 tf],[Vc3L_ref-delta Vc3L_ref-delta],'k-'),zoom
title('Barramento $E_a$','FontSize',18,'Interpreter','latex')
legend('$E_a$','$E_a^*$','FontSize',16,'Interpreter','latex','Location','northwest')
xlabel('Tempo (s)','FontSize',14)
ylabel('Tensão (V)','FontSize',14)
grid('minor')
axis(ax1);

ax2 = [0 3 140 240];
sp2 = subplot(3,1,2);
plot(Ts,Vc2Ls,Ts,Vc2L_refs,'r--',...
    [tsave0 tf/2],[Vc2L_ref+delta Vc2L_ref+delta],'k-',...
    [tsave0 tf/2],[Vc2L_ref-delta Vc2L_ref-delta],'k-'),zoom
title('Barramento $E_b$','FontSize',18,'Interpreter','latex')
legend('$E_b$','$E_b^*$','FontSize',16,'Interpreter','latex','Location','northwest')
xlabel('Tempo (s)','FontSize',14)
ylabel('Tensão (V)','FontSize',14)
grid('minor')
axis(ax2);

ax3 = [0 3 -200 200];
sp3 = subplot(3,1,3);
plot(Ts,vsb10_meds-vsb20_meds,'b-'),zoom
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




figure('name','Histerese e vsh_med') %vsh
ax4 = [0 1.49 170 190];
sp2 = subplot(1,2,1);
plot(Ts,Vc2Ls,Ts,Vc2L_refs,'r--','LineWidth',2)
hold on
plot([tsave0 tf/2],[Vc2L_ref+delta Vc2L_ref+delta],'k-',...
    [tsave0 tf/2],[Vc2L_ref-delta Vc2L_ref-delta],'k-'),zoom
title('Barramento $E_b$ Controlado por Histerese','FontSize',18,'Interpreter','latex')
legend('$E_b$','$E_b^*$','FontSize',16,'Interpreter','latex','Location','northwest')
xlabel('Tempo (s)','FontSize',14)
ylabel('Tensão (V)','FontSize',14)
grid('minor')
axis(ax4);

ax5 = [0 1.49 -200 200];
sp3 = subplot(1,2,2);
plot(Ts,vsb10_meds-vsb20_meds,'b-','LineWidth',2),zoom
title('Tens\~{a}o $v_{sb}$','FontSize',18,'Interpreter','latex')
legend('$v_{sb}$','FontSize',16,'Interpreter','latex','Location','northwest')
xlabel('Tempo (s)','FontSize',14)
ylabel('Tensão (V)','FontSize',14)
grid('minor')
axis(ax5);

