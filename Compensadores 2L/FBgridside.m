% Compensador 3L
% Ademar Alves

clear;clc;close all;
tic
  
%% Condições do Sistema
SwSg = 1.0;              % Sobretensão ou afundamento
Vgm = 220*sqrt(2)*SwSg;  % Amplitude da tensão da rede
Vel_ref = 220*sqrt(2);   % Amplitude da tensão de referência sobre a carga

thetal_ref = 0;          % Fase da tensão de referência braço L

f_ref = 60;              % Frequência de referência
w_ref = 2*pi*f_ref;      % Frequência angular de referência

% Barramento 3L 
m = 0.5;
Vc3L_ref = 340;
% Vc3L_ref = 110*sqrt(2)/m;  % Tensão de referência do barramento 3L [V]
Vc3L = 0*Vc3L_ref;           % Tensão medida no barramento 3L         
Ea = Vc3L_ref;               % Tensão de referência nominal do barramento 3L

Cap = 8.8E-3;            % Capacitância do barramento

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

% %   SHUNT
% % Dados do Shunt
Rs = 0.2;                % Resistência
Ls = 7e-3;               % Indutância
Xs = Ls*w_ref;           % Reatância

%   REDE
% Rg = 0.2;                % Resistência
% Rg = 0.1;
%Lg = 7e-3;               % Indutância
% Lg = 5e-3;
% Xg = Lg*w_ref;           % Reatância

% Fator de potência
FPg = 1;                 % Fator de potência da rede
thetag = acos(FPg);      % Defasagem tensão-corrente da rede


%% Parâmetros de Simulação
h = 1E-7;                % Passo de cálculo
t = 0;                   % Tempo inicial de simulação
tf = 2.5;                % Tempo final de simulação


%% Parâmetros de Gravação
% tsave0 = tf-1/60;
tsave0 = 0;
tsave = tsave0;          % Tempo de gravação
% npt = 200000;          % Dimensão do vetor de saída de dados

% hsave = (tf-tsave0)/npt; % Passo de de gravação dos vetores de saída de dados
hsave = 10^-6;
npt = (tf-tsave0)/hsave;
if hsave < h             % Sendo o Passo de gravação menor que o passo de cálculo (Saturação)
    hsave = h;           % Defina o passo de gravação = passo de cálculo
    npt = (tf-tsave0)/hsave; % Recalcule o a dimensão dos vetores de saída de dados
end

n = 0;                   % Inicialização da variável de posição dos vetores de saída

%% Inicialização das variáveis
vsh = 0;                 % Tensão Shunt Gerada

ig = 0;                  % Corrente inicial no ramo g
il = 0;                  % Corrente inicial no ramo l
is = 0;                  % Corrente inicial no ramo s
if0 = 0;

% 3L
vsa0_int = 0;            % valor inicial da integral da tensão no polo sa
vga0_int = 0;            % valor inicial da integral da tensão no polo ga
vla0_int = 0;            % valor inicial da integral da tensão no polo la

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

%% Cálculo de Regime Permanente
    %% Cálculo da Amplitude da corrente drenada pela rede
%     fasegl = cos(thetal_ref - acos(FPl));
%     Igc = min(roots([Rg/2, -(Vgm/2 + Rg*Il*fasegl), Pl+(Rg*Il^2)/2]));

    
    %% Amplitude  e Fase da tensão gerada na compensação shunt
%     Is     = sqrt(Igc^2 + Il^2 - 2*Igc*Il * fasegl);
%     faseIs = acos((Igc^2 + Is^2 - Il^2)/(2*Igc*Is));
%     Vsh    = Vgm - (Rg + 1i*Xg)*Ig*(cos(faseIs)+1i*sin(faseIs));
% 
%     Vshref = abs(Vsh);
%     fasesh = angle(Vsh);

    
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

%% PI ressonante
Imax = 50;
kii = 100.0;
kpi = 10.;

kic = 20.;
kpc = 0.5;

hpwm = 1/10000;
iw = 0.0027;

arg = w_ref*hpwm;
F1 = cos(arg);
F2 = sin(arg)*iw;
F3 = 2.0*kii*sin(arg)*iw; 
F4=-w_ref*sin(arg);
F5=cos(arg);
F6=(cos(arg)-1.0)*2.0*kii;

% Inicialização
ei1 = 0;
ei1a = 0;
xaa1 = 0;
xa1 = 0;
xb1 = 0;

%--------------------------------------------------------------------
%% Início do Looping
while t<tf
    t = t + h;
    
%     if t > tf/2
%         sobretensao = 1;
%     end 

% % SOBRETENSÃO
%     if sobretensao == 1
%         Vgm = 1.5*110*sqrt(2);
%     end
    
%     if t>1.5
%        if0 = 4; 
%         
%     end
    eg = Vgm*cos(w_ref*t);    % Tensão da rede elétrica
    
    %% Onda triangular
    if t >= ttri
        ttri = ttri + htri;    % Atualização da onda triangular
        
        %% Controlador PI da corrente ig

        Vc3L_error = Vc3L_ref - Vc3L;            % Erro controle barramento
        P_error = Kp*Vc3L_error;                 % Erro proporcional
        I_error = I_error + htri*Ki*Vc3L_error;  % Erro integral

%       Saturador
%         if(I_error > Imax) 
%             I_error = Imax;
%         elseif(I_error < -Imax)
%             I_error = -Imax;
%         end
        
        Ig = P_error + I_error + Ig0;            % Amplitude ig*
        
%       Saturador
%         if(Ig > Imax) 
%             Ig = Imax;
%         elseif(Ig < -Imax)
%             Ig = -Imax;
%         end
        
%         Ig = sqrt(2)*Pl/(Vgm/sqrt(2));
        
        ig_ref = Ig*cos(w_ref*t - thetag);       % ig*
        ig_error = (ig_ref - ig);                % Erro ig
        
        ig_error_ressonante = -ig_error;         % Erro ig ressonante
        
        %% Correntes de Regime Permanente            
        il_ref = Il*cos(w_ref*t - thetal);
        is_ref = ig_ref - il_ref;
        is_error = (is_ref  - is);
        

        %% PI ressonante
        ei1a = ei1;
        xaa1 = xa1;

        ei1 = ig_error_ressonante;

        xa1 = F1*xaa1+F2*xb1+F3*ei1a;
        xb1 = F4*xaa1+F5*xb1+F6*ei1a;	
        
        if(xa1 >  Ea) 
            xa1 =  Ea;
        end
        
        if(xa1 < -Ea) 
            xa1 = -Ea;
        end
        
%         vg_ref = xa1 + 2.*kpi*ei1;
        
%       Saturador
%         if(vg_ref >  Ea) 
%             vg_ref =  Ea;
%         end
%         if(vg_ref < -Ea) 
%             vg_ref = -Ea;
%         end
        
        %% Tensões de referência
%         vg_ref = eg - Rg*ig - (Lg/htri)*ig_error;        
%         vl_ref = Vel_ref*cos(w_ref*t + thetal_ref);
        vsh_ref = eg - Rs*is - (Ls/htri)*is_error;
        vel_ref = Vel_ref*cos(w_ref*t + thetal_ref);        
        vse_ref = eg - vel_ref;
        
%         vsh_ref_error = vsh_ref - vsh;
        
        %% Cálculo das tensões de polo

%         Vs = [vl_ref,vg_ref,0];
%         Vs = [vse_ref,vsh_ref,0];
%         vxmax =  Vc3L_ref/2 - max(Vs);
%         vxmin = -Vc3L_ref/2 - min(Vs);
%         vx = (vxmax+vxmin)/2;

%         vsa0_ref = vx;
%         vga0_ref = vg_ref + vsa0_ref;
%         vla0_ref = vl_ref + vsa0_ref;   
        
        vga0_ref = vse_ref;
        vsa0_ref = vga0_ref - vsh_ref;
        
        %% Tensões médias nos braços
        % 3L
        vsa0_med = vsa0_int/htri;
        vga0_med = vga0_int/htri;
%         vla0_med = vla0_int/htri;
        
        vsa0_int = 0;
        vga0_int = 0;
%         vla0_int = 0;
        
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
    
%     if vla0_ref >= vt3L % Estado das chaves (Braço l)
%         qla = 1; % Chave fechada (IGBT em condução)
%     else
%         qla = 0; % Chave aberta (IGBT em corte)
%     end
    
    %% Tensões de polo
    % 3L
    vsa0 = (2*qsa - 1)*(Vc3L/2);
    vga0 = (2*qga - 1)*(Vc3L/2);
%     vla0 = (2*qla - 1)*(Vc3L/2);
    
    %% Tensões geradas
%     vsh = vga0 - vsa0;
%     vse = vga0 - vla0;
%     el = eg - vse;

%     vg = vga0 - vsa0;
%     vl = vla0 - vsa0;
    
    vse = vga0;
    vsh = vga0 - vsa0;
    el = eg - vse;
    
    %% Integração das tensões de polo
    % 3L
    vsa0_int = vsa0_int + vsa0*h;
    vga0_int = vga0_int + vga0*h;
%     vla0_int = vla0_int + vla0*h;  
    
    % Carga
%     el_int = el_int + el*h;

%     el_int = el_int + vl*h;
    el_int = el_int + el*h;
    
    %% Integração numérica das correntes ig e il
    il = il*(1 - h*Rl/Ll) + (h/Ll)*(eg - vse);
    is = is*(1 - h*Rs/Ls) + (h/Ls)*(eg - vsh);
    ig = is + il;

%     il = il*(1 - h*Rl/Ll) + (h/Ll)*(vl);
%     ig = ig*(1 - h*Rg/Lg) + (h/Lg)*(eg - vg);
%     is = ig - il;
    
    %% Controle do barramento
    % 3L
    Rp3L = 5e10;
    Rs3L = 0.0;
%     ik3L = ig*qga - il*qla - is*qsa + if0;
    ik3L = ig*qga - is*qsa;
%     ik3L = ig_ref*qga - il_ref*qla - is_ref*qsa;
    ic3L = (Rp3L*ik3L - Vc3L)/(Rp3L + Rs3L);   % Corrente de entrada barramento 2L
    Vc3L = Vc3L + (h/Cap)*ic3L;                % Tensão barramento 3L
%     Vc3L = Vc3L_ref*1.00;
    

    %% Potências
    Peg = eg*ig;
    Pel = el_med*il;
    Pfc = if0*Vc3L;
    
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
%         qls(n) = qla;
        
        % Tensões de polo de referência
        vs0_refs(n) = vsa0_ref;
        vg0_refs(n) = vga0_ref;
%         vl0_refs(n) = vla0_ref;
        
        % Tensões de referência 
        vsh_refs(n) = vsh_ref;
        vse_refs(n) = vse_ref;  
        vel_refs(n)= vel_ref; 

%         vg_refs(n) = vg_ref;
%         vl_refs(n) = vl_ref;

        % Tensões de polo
        vs0s(n) = vsa0;
        vg0s(n) = vga0;
%         vl0s(n) = vla0;
        
        % Tensões geradas
        vshs(n) = vsh;
        vses(n) = vse;    
        els(n) = el; % Carga

%         vgs(n) = vg;
%         vls(n) = vl;    

        % Tensões médias
        vs0_meds(n) = vsa0_med;
        vg0_meds(n) = vga0_med;
%         vl0_meds(n) = vla0_med;   
        el_meds(n) = el_med; % Carga
        
        % Corrente de referência do controlador
        ig_refs(n) = ig_ref;
        
        % Correntes
        iss(n) = is;
        igs(n) = ig;
        ils(n) = il;
        
        % Barramento
        Vc3Ls(n) = Vc3L;
        Vc3L_refs(n) = Vc3L_ref;
        
        % Portadora triangular
        vtris(n) = vtri;
        vt3Ls(n) = vt3L;   
        
        % Correntes de referência
        ig_refs(n) = ig_ref;
        il_refs(n) = il_ref;
%         is_refs(n) = is_ref;
        
        % Erros
        ig_errors(n) = ig_error;
%         vsh_ref_errors(n) = vsh_ref_error;
        
        % Potências
        Pegs(n) = Peg;
        Pels(n) = Pel;
        Pfcs(n) = Pfc;
    end
end

toc

%% Plots

% ---- Tensões Vg e Vl
figure('name','Tensão Vg') %vsh
plot(Ts,vsh_refs,...
     Ts,vg0_meds-vs0_meds,...
     Ts,vshs),zoom
% plot(Ts,vg_refs,...
%      Ts,vg0_meds-vs0_meds,...
%      Ts,vgs),zoom
title('Tensão vg','FontSize',18)
legend('vg_{ref}','vg_{med}','vg_{pwm}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")
% axis([0 tf -(Vc3L_ref*2-20) (Vc3L_ref*2+20)])

figure('name','Tensão Vl') %vse
plot(Ts,vse_refs,...
     Ts,vg0_meds,...
     Ts,vses),zoom
%      Ts,vg0_meds-vl0_meds,...

% plot(Ts,vl_refs,...
%      Ts,vl0_meds-vs0_meds,...
%      Ts,vls),zoom
title('Tensão vl','FontSize',18)
legend('vl_{ref}','vl_{med}','vl_{pwm}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")
% axis([0 tf -Vc3L_ref-20 Vc3L_ref+20])

% ---- Tensão na carga
figure('name','Tensão Vel') %vsh
plot(Ts,vel_refs,...
     Ts,el_meds,...
     Ts,els),zoom
title('Tensão vel','FontSize',18)
legend('vel_{ref}','vel_{med}','vel_{pwm}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")
axis([0 tf -(Vc3L_ref*2-20) (Vc3L_ref*2+20)])

% ---- Correntes
figure('Name','Corrente do circuito: Lado G, S e L')
% plot(Ts,igs,Ts,ils,'r-','LineWidth',1),zoom
plot(Ts,igs,Ts,ils,Ts,iss,'r-','LineWidth',1),zoom
title('Corrente do circuito: ig, il e ia')
% title('Corrente do circuito: ig e il')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
% legend('ig','il','is','FontSize',18)
legend('ig','il','is','FontSize',18)
grid()

% ---- Correntes de Referência
% figure('Name','Corrente de Referência do circuito: Lado G, S e L')
% plot(Ts,ig_refs,Ts,il_refs,Ts,is_refs,'r-','LineWidth',1),zoom
% title('Corrente de Referência do circuito: ig, il e ia')
% xlabel("Tempo (s)")
% ylabel("Corrente (A)")
% legend('ig','il','is','FontSize',18)
% grid()

% ---- Barramento
figure('name','Tensão no barramento 3L')
plot(Ts,Vc3Ls,Ts,Vc3L_refs,'r--')
title('Tensão do barramento 3L','FontSize',18)
legend('Vdc3L_{med}','Vdc3L_{ref}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid('minor')

% ---- Estados da chaves
% figure('name','Estado da chave qga')
% plot(Ts,100*qgs,Ts, vg0_refs,Ts,vt3Ls,'r-'),zoom
% title('Estado da chave qga','FontSize',18)
% legend('Chave $q_{ga}$','$v_{ga0_{ref}}$','$v_{3L_{tri}}$','FontSize',16,'Interpreter','latex')
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% grid('minor')

% figure('name','Estado da chave qla')
% plot(Ts,100*qls,Ts, vl0_refs,Ts,vt3Ls,'r-'),zoom
% title('Estado da chave qla','FontSize',18)
% legend('Chave $q_{la}$','$v_{la0_{ref}}$','$v_{3L_{tri}}$','FontSize',16,'Interpreter','latex')
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% grid('minor')

figure('name','Estado da chave qsa')
plot(Ts,100*qss,Ts, vs0_refs,Ts,vt3Ls,'r-'),zoom
title('Estado da chave qsa','FontSize',18)
legend('Chave $q_{sa}$','$v_{sa0_{ref}}$','$v_{3L_{tri}}$','FontSize',16,'Interpreter','latex')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid('minor')

% figure('name','Triangulares')
% plot(200*vtris,'r-'),zoom
% title('Portadoras triangulares','FontSize',18)
% legend('$vtri$','FontSize',16,'Interpreter','latex')
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% grid('minor')

% thd = thdf(igs,1/h,f_ref)
% thdf(ils,1/h,f_ref)

figure('name','Potências')
plot(Ts,Pegs,Ts,Pels,Ts,Pfcs,'r-'),zoom
title('Potências','FontSize',18)
legend('Peg','Pel','Pfc','FontSize',16,'Interpreter','latex')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid('minor')

a = ones(1,length(Ts));

figure('name','Potências CTE')
% plot(Ts,a*mean(Pegs(n-10000:n)),Ts,a*mean(Pels(n-10000:n)),Ts,a*mean(Pfcs(n-10000:n)),'r-'),zoom
plot(Ts,a*mean(Pegs(n-100000:n)),Ts,a*mean(Pels(n-100000:n)),Ts,a*mean(Pfcs(n-100000:n)),'r-'),zoom
title('Potências CTE','FontSize',18)
legend('Peg','Pel','Pfc','FontSize',16,'Interpreter','latex')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid('minor')

figure('name','Tensão e corrente da Rede');plot(Ts,egs,Ts,igs*20,Ts,ig_refs*20)