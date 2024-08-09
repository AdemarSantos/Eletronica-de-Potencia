% Full Bridge: Load-Side
% Ademar Alves

clear;clc;close all;
tic
  
%% Condições do Sistema
SwSg = 1.0;              % Sobretensão ou afundamento
Vgm = 110*sqrt(2)*SwSg;  % Amplitude da tensão da rede
Vel_ref = 110*sqrt(2);   % Amplitude da tensão de referência sobre a carga

thetal_ref = 0;          % Fase da tensão de referência braço L

f_ref = 60;              % Frequência de referência
w_ref = 2*pi*f_ref;      % Frequência angular de referência

% Barramento 2L 
m = 0.5;
Vc2L_ref = 360;
% Vc2L_ref = 110*sqrt(2)/m;  % Tensão de referência do barramento 2L [V]
Vc2L = 1*Vc2L_ref;           % Tensão medida no barramento 2L         
Ea = Vc2L_ref;               % Tensão de referência nominal do barramento 2L

Cap = 8.8E-3;            % Capacitância do barramento

%% Impedâncias de entrada e saída

%   CARGA
% Dados da carga
Pl = 2000;               % Potência ativa consumida na carga [W]
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
Rs = 0.1;                % Resistência
Ls = 5e-3;               % Indutância
Xs = Ls*w_ref;           % Reatância

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

% 2L
vsa0_int = 0;            % valor inicial da integral da tensão no polo sa
vla0_int = 0;            % valor inicial da integral da tensão no polo la

% Carga
el_int = 0;              % Valor inicial da integral da tensão na carga

control = 1;
cont1 = 0;
cont2 = 0;

k=0;                     % Índice de salvamento da potência

%% Parâmetros do controlador 
% Controlador PI
Kp = 0.5; %0.2
Ki = 20;  %15

P_error = 0;
I_error = 0;

vc_ref = Vc2L;
vc2L = Vc2L;

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
Imax = 40;
kii = 1000.0;
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
    eg = Vgm*cos(w_ref*t);    % Tensão da rede elétrica
    
    %% Onda triangular
    if t >= ttri
        ttri = ttri + htri;    % Atualização da onda triangular
        
        %% Controlador PI da corrente ig

        Vc2L_error = Vc2L_ref - Vc2L;            % Erro controle barramento
        P_error = Kp*Vc2L_error;                 % Erro proporcional
        I_error = I_error + htri*Ki*Vc2L_error;  % Erro integral

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
%             Ig = 0;
%         end
        
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
        
        vsh_ref = xa1 + 2.*kpi*ei1; % Ressonante
        
%       Saturador
%         if(vsh_ref >  Ea) 
%             vsh_ref =  Ea;
%         end
%         if(vsh_ref < -Ea) 
%             vsh_ref = -Ea;
%         end
        
        %% Tensões de referência
%         vsh_ref = eg - Rs*is - (Ls/htri)*is_error; % Preditivo
        vel_ref = Vel_ref*cos(w_ref*t + thetal_ref);        
        vse_ref = eg - vel_ref;
        
        %% Cálculo das tensões de polo
        vla0_ref = -vse_ref;
        vsa0_ref = -vsh_ref;
        
        %% Tensões médias nos braços
        % 2L
        vsa0_med = vsa0_int/htri;
        vla0_med = vla0_int/htri;
        
        vsa0_int = 0;
        vla0_int = 0;
        
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
    vt2L = vtri*Vc2L;
    
    %% Estado das chaves
    % 2L
    if vsa0_ref >= vt2L % Estado das chaves (Braço s)
        qsa = 1; % Chave fechada
    else
        qsa = 0; % Chave aberta
    end
    
    if vla0_ref >= vt2L % Estado das chaves (Braço l)
        qla = 1; % Chave fechada
    else
        qla = 0; % Chave aberta
    end
    
    %% Tensões de polo
    % 2L
    vsa0 = (2*qsa - 1)*(Vc2L/2);
    vla0 = (2*qla - 1)*(Vc2L/2);
    
    %% Tensões geradas
    vse = -vla0;
    vsh = -vsa0;
    el = eg - vse;
    
    %% Integração das tensões de polo
    % 2L
    vsa0_int = vsa0_int + vsa0*h;
    vla0_int = vla0_int + vla0*h;
    
    el_int = el_int + el*h;
    
    %% Integração numérica das correntes ig e il
    il = il*(1 - h*Rl/Ll) + (h/Ll)*(eg - vse);
    is = is*(1 - h*Rs/Ls) + (h/Ls)*(eg - vsh);
    ig = is + il;
    
    %% Controle do barramento
    % 2L
    Rp2L = 5e10;
    Rs2L = 0.0;
    ik2L = -il*qla - is*qsa;
    ic2L = (Rp2L*ik2L - Vc2L)/(Rp2L + Rs2L);   % Corrente de entrada barramento 2L
    Vc2L = Vc2L + (h/Cap)*ic2L;                % Tensão barramento 2L
%     Vc2L = Vc2L_ref*1.00;
    
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
        qls(n) = qla;
        
        % Tensões de polo de referência
        vs0_refs(n) = vsa0_ref;
        vl0_refs(n) = vla0_ref;
        
        % Tensões de referência 
        vsh_refs(n) = vsh_ref;
        vse_refs(n) = vse_ref;  
        vel_refs(n)= vel_ref; 

        % Tensões de polo
        vs0s(n) = vsa0;
        vl0s(n) = vla0;
        
        % Tensões geradas
        vshs(n) = vsh;
        vses(n) = vse;    
        els(n) = el; % Carga

        % Tensões médias
        vs0_meds(n) = vsa0_med;
        vl0_meds(n) = vla0_med;   
        el_meds(n) = el_med; % Carga
        
        % Corrente de referência do controlador
        ig_refs(n) = ig_ref;
        il_refs(n) = il_ref;
        is_refs(n) = is_ref;
        
        % Correntes
        iss(n) = is;
        igs(n) = ig;
        ils(n) = il;
        
        % Barramento
        Vc2Ls(n) = Vc2L;
        Vc2L_refs(n) = Vc2L_ref;
        
        % Portadora triangular
        vtris(n) = vtri;
        vt2Ls(n) = vt2L;   
        
        % Correntes de referência
        ig_refs(n) = ig_ref;
        il_refs(n) = il_ref;
        
        % Erros
        ig_errors(n) = ig_error;
        
    end
end

toc

%% Plots

% ---- Tensões Vg e Vl
figure('name','Tensão Vsh') %vsh
plot(Ts,vsh_refs,...
     Ts,-vs0_meds,...
     Ts,vshs),zoom
title('Tensão vsh','FontSize',18)
legend('vsh_{ref}','vsh_{med}','vsh_{pwm}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")
% axis([0 tf -Vc2L_ref-20 Vc2L_ref+20])

figure('name','Tensão Vse') %vse
plot(Ts,vse_refs,...
     Ts,-vl0_meds,...
     Ts,vses),zoom
title('Tensão vse','FontSize',18)
legend('vse_{ref}','vse_{med}','vse_{pwm}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")
% axis([0 tf -Vc2L_ref-20 Vc2L_ref+20])

% ---- Tensão na carga
figure('name','Tensão Vel') %vsh
plot(Ts,vel_refs,...
     Ts,el_meds,...
     Ts,els),zoom
title('Tensão vel','FontSize',18)
legend('vel_{ref}','vel_{med}','vel_{pwm}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")
axis([0 tf -(Vc2L_ref*2-20) (Vc2L_ref*2+20)])

% ---- Correntes
figure('Name','Corrente do circuito: Lado G, S e L')
plot(Ts,igs,Ts,ils,Ts,iss,'r-','LineWidth',1),zoom
title('Corrente do circuito: ig, il e is')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
legend('ig','il','is','FontSize',18)
grid()

% ---- Correntes de Referência
figure('Name','Corrente de Referência do circuito: Lado G, S e L')
plot(Ts,ig_refs,Ts,il_refs,Ts,is_refs,'r-','LineWidth',1),zoom
title('Corrente de Referência do circuito: ig, il e ia')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
legend('ig_{ref}','il_{ref}','is_{ref}','FontSize',18)
grid()

% ---- Barramento
figure('name','Tensão no barramento 2L')
plot(Ts,Vc2Ls,Ts,Vc2L_refs,'r--')
title('Tensão do barramento 2L','FontSize',18)
legend('Vdc2L_{med}','Vdc2L_{ref}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid('minor')

% ---- Estados da chaves
% figure('name','Estado da chave qga')
% plot(Ts,100*qgs,Ts, vg0_refs,Ts,vt2Ls,'r-'),zoom
% title('Estado da chave qga','FontSize',18)
% legend('Chave $q_{ga}$','$v_{ga0_{ref}}$','$v_{2L_{tri}}$','FontSize',16,'Interpreter','latex')
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% grid('minor')

% figure('name','Estado da chave qla')
% plot(Ts,100*qls,Ts, vl0_refs,Ts,vt2Ls,'r-'),zoom
% title('Estado da chave qla','FontSize',18)
% legend('Chave $q_{la}$','$v_{la0_{ref}}$','$v_{2L_{tri}}$','FontSize',16,'Interpreter','latex')
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% grid('minor')

figure('name','Estado da chave qsa')
plot(Ts,100*qss,Ts,vs0_refs,Ts,vt2Ls,'r-'),zoom
title('Estado da chave qsa','FontSize',18)
legend('Chave $q_{sa}$','$v_{sa0_{ref}}$','$v_{2L_{tri}}$','FontSize',16,'Interpreter','latex')
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