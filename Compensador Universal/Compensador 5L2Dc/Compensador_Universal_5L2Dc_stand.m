% Conversor 5L2D com controle da corrente da rede e dos barramentos

clear;close all;clc

%---------------- Impedâncias de entrada e saída ------------------
Rg = 0.25;               % Resistência de entrada
Lg = 3.5E-3;             % Indutância de entrada
Rl = 15;                 % Resistência da carga
Ll = 15E-3;              % Indutância da carga

%---------------- Parâmetros de Simulação ------------------
h = 1E-7;                % Passo de cálculo
t = 0;                   % Tempo inicial de simulação
tf = 1;

%---------------- Parâmetros de Gravação ------------------
tsave0 = 0.;             % Tempo inicial de gravação
tsave = tsave0;          % Tempo de gravação
npt = 50000;            % Dimensão do vetor de saída de dados

hsave = (tf-tsave0)/npt; % Passo de de gravação dos vetores de saída de dados

if hsave < h             % Sendo o Passo de gravação menor que o passo de cálculo (Saturação)
    hsave = h;           % Defina o passo de gravação = passo de cálculo
    npt = (tf-tsave0)/hsave; % Recalcule o a dimensão dos vetores de saída de dados
end
n = 0;                   % Inicialização da variável de posição dos vetores de saída

%---------------- Condições do Sistema ------------------
Vg_ref = 90;             % Amplitude da tensão de referência braço G
Vl_ref = 90;             % Amplitude da tensão de referência braço L

%Vgm = 123.8;            % Amplitude da tensão de entrada (eg)
%Vgm = 50:1:110;
Vgm = 90;                % Amplitude da tensão de entrada (eg)

alpha = pi/4;            % Fase da tensão de referência braço L

f_ref = 60;              % Frequência de referência
w_ref = 2*pi*f_ref;      % Frequência angular de referência

ig = 0;                  % Corrente inicial no ramo g
il = 0;                  % Corrente inicial no ramo l

vgb10b_int = 0;          % valor inicial da integral da tensão no polo gb1
vgb20b_int = 0;          % valor inicial da integral da tensão no polo gb2
vha0a_int = 0;           % valor inicial da integral da tensão no polo ha
vga0a_int = 0;           % valor inicial da integral da tensão no polo ga
vla0a_int = 0;           % valor inicial da integral da tensão no polo la

Pot2L_int = 0;           % valor inicial da integral da potência no barramento 2L
Pot3L_int = 0;           % valor inicial da integral da potência no barramento 3L

vx = 0;
vy = 0;

%---------------- Parâmetros do controlador ------------------
Ki = 15;                 % Coeficiente integrativo
Kp = 0.2;                % Coeficiente proporcional
I0 = 3;                  % Amplitude inicial

I_error = 0;             % Erro integral
P_error = 0;             % Erro proporcional

% Barramento 3L
Vc3L_ref = 100;          % Tensão de referência do barramento 3L (Conv. A)
Vc3L = Vc3L_ref;         % Tensão medida no barramento 3L (Conv. A)          

% Barramento 2L
Vc2L_ref = 100;          % Tensão de referência do barramento 2L (Conv. B)
Vc2L = Vc2L_ref;         % Tensão medida no barramento 2L (Conv. B)     

delta = Vc2L_ref*0.05;   % Histerese
Cap = 4.4E-3;            % Capacitância de barramento

%---------------- Inicialização da portadora triangular ------------------
ttriangle = 0;           % Tempo inicial da onda triangular
ftriangle = 10E3;        % Frequência da onda triangular de 10 kHz
htriangle = 1/ftriangle; % Período da onda triangular
Vt = Vc3L_ref/2;         % Tensão de pico da onda triangular
vtriangle = Vt;          % Tensão máxima da onda triangular
dtriangle = 2*Vt/htriangle;% Derivada da onda triangular (dV/dT)
sign = -1;               % Sinal inicial da derivada (Comportamento decrescente)

%%%%%%%%%%%%%%%%%%%%%%%%%%
cont1=0;
cont2=0;
cont3=0;

Pot2Lmax = 0;
Pot2Lmin = 0;

Pot2Lmax_total = 0;
Pot2Lmin_total = 0;

pot2Lmax_total_int = 0;
pot2Lmin_total_int = 0;

Potc = 0;
Potd = 0;

tete = 0;

control=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------------------------------------------------------------
while t<tf
    t = t + h;
    
    eg = Vgm*cos(w_ref*t);                     % Tensão da rede
    
    %%%%%%%%%%%%%%%%%
    % Onda triangular
    if t >= ttriangle
        ttriangle = ttriangle + htriangle;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Contolador PI da corrente da rede
        Vc3L_error = Vc3L_ref - Vc3L;
        P_error = Kp*Vc3L_error;
        I_error = I_error + htriangle*Ki*Vc3L_error;
        Ig = P_error + I_error + I0;
        
        ig_ref = Ig*cos(w_ref*t);
        ig_error = ig_ref - ig;
        
        %%%%%%%%%%%%%%%%%%%%%%%
        % Tensões de referência       
        %vg_ref = Vg_ref*cos(w_ref*t);
        vg_ref = eg - ig*Rg - (Lg/htriangle)*ig_error;
        vl_ref = Vl_ref*cos(w_ref*t + alpha);        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Cálculo das tensões de polo
        vxmaxgba =  (Vc2L+Vc3L/2) - vg_ref;
        vxmingba = -(Vc2L+Vc3L/2) - vg_ref;
        vxmaxl0  =  Vc3L/2 - max([vl_ref,0]);
        vxminl0  = -Vc3L/2 - min([vl_ref,0]);
        vxmax = min([vxmaxgba,vxmaxl0]);
        vxmin = max([vxmingba,vxminl0]);
        vx = (vxmin+vxmax)/2;
        
        vgba_ref  = vg_ref + vx;

        vymaxga0 =  Vc3L/2 - vgba_ref/2;
        vyminga0 = -Vc3L/2 - vgba_ref/2;
        vymaxgb  =  Vc2L + vgba_ref/2;
        vymingb  = -Vc2L + vgba_ref/2;
        vymax = min([vymaxga0,vymaxgb]);
        vymin = max([vyminga0,vymingb]);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Controle do barramento 2L por histerese
        Vc2L_error = Vc2L_ref - Vc2L;
        
        if Vc2L_error >= delta 
            control = 1;     % Carregar
        elseif Vc2L_error <= -delta
            control = -1;    % Descarregar
        end
        
        if     control == 1  % Carregar
            cont1 = cont1+1;
            if ig >= 0
                vy = vymin+vymax;
            else
                vy = vymax+vymin;
            end
            
        elseif control == -1 % Descarregar
            cont2 = cont2+1;
            if ig >= 0
                vy = vymax;
            else
                vy = vymin;
            end
            
        end

        vgb_ref = -vgba_ref/2 + vy;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Tensões de polo de referência
        % 2L
        vgb10b_ref = -vgb_ref/2;
        vgb20b_ref = vgb_ref/2;
        
        % 3L
        vga0a_ref = vgba_ref/2 + vy;
        vla0a_ref = vl_ref + vx;
        vha0a_ref = vx;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Tensões médias nos braços
        % 2L
        vgb10b_med = vgb10b_int/htriangle;
        vgb20b_med = vgb20b_int/htriangle;
        
        vgb10b_int = 0;
        vgb20b_int = 0;
        
        % 3L
        vha0a_med = vha0a_int/htriangle;
        vga0a_med = vga0a_int/htriangle;
        vla0a_med = vla0a_int/htriangle;
        
        vha0a_int = 0;
        vga0a_int = 0;
        vla0a_int = 0;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Potência média nos barramentos
        % 2L
        Pot2L_med = Pot2L_int/htriangle;
        Pot2L_int = 0;
        
        % 3L
        Pot3L_med = Pot3L_int/htriangle;
        Pot3L_int = 0;
        
        %%%%%%%%%%%%%%%%%%%%%%
        % Portadora triangular
        if vtriangle <= 0
           vtriangle = -(Vt);
           sign = 1;
        else
            vtriangle = (Vt);
            sign = -1;
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Progressão da portadora triangular
    vtriangle = vtriangle + sign*dtriangle*h;
    
    %%%%%%%%%%%%%%%%%%%
    % Estado das chaves
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
        
    %%%%%%%%%%%%%%%%%
    % Tensões de polo
    % 2L
    vgb10b = (2*qgb1 - 1)*(Vc2L/2); 
    vgb20b = (2*qgb2 - 1)*(Vc2L/2);
    
    % 3L
    vha0a = (2*qha - 1)*(Vc3L/2);
    vga0a = (2*qga - 1)*(Vc3L/2);
    vla0a = (2*qla - 1)*(Vc3L/2);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Integração das tensões de polo
    % 2L
    vgb10b_int = vgb10b_int + vgb10b*h;
    vgb20b_int = vgb20b_int + vgb20b*h;
    
    % 3L
    vha0a_int = vha0a_int + vha0a*h;
    vga0a_int = vga0a_int + vga0a*h;
    vla0a_int = vla0a_int + vla0a*h;
    
    %%%%%%%%%%%%%%%%%
    % Tensões de fase
    vgha = vga0a - vha0a;
    vgb = vgb10b - vgb20b;
    
    vg = vgb + vgha;
    vl = vla0a - vha0a;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Integração numérica das correntes ig e il
    ig = ig*(1 - h*Rg/Lg) + (h/Lg)*(eg - vg);  % Corrente do ramo g
    il = il*(1 - h*Rl/Ll) + (h/Ll)*vl;         % Corrente do ramo l   
    ih = ig - il;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Controle do barramento
    % 2L
    ic2L = ig*qgb1 - ig*qgb2;
    Vc2L = Vc2L + (h/Cap)*ic2L;
    %Vc2L = Vc2L_ref*1.00;
    % 3L
    ic3L = ig*qga - il*qla - ih*qha;
    Vc3L = Vc3L + (h/Cap)*ic3L;
    %Vc3L = Vc3L_ref*1.00;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Potência nos barramentos
    % 2L
    Pot2L = ic2L*Vc2L;
    Pot2L_int = Pot2L_int + Pot2L*h;

    % 3L
    Pot3L = ic3L*Vc3L;
    Pot3L_int = Pot3L_int + Pot3L*h;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Potência de carregamento e descarregamento por ciclo de 60 Hz
    tete = tete + h*w_ref;
    
    if     control == 1  % Carregar
        Pot2Lmax = Pot2Lmax + vgb*ig;
        
    elseif control == -1 % Descarregar
        Pot2Lmin = Pot2Lmin + vgb*ig;
    end
    
    if tete >= 2*pi
        cont3 = cont3+1;
        tete = tete - 2*pi;
        Pot2Lmax_total = Pot2Lmax;
        Pot2Lmin_total = Pot2Lmin;
        Pot2Lmax = 0;
        Pot2Lmin = 0;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Potência de carregamento e descarregamento
    if     control == 1  % Carregar
        Potc = vgb*ig;
        Potd = 0;
    elseif control == -1 % Descarregar
        Potd = vgb*ig;
        Potc = 0;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Salvamento das variáveis
    if tsave <= t
        % Variáveis de salvamento
        tsave = tsave + hsave;
        n = n + 1;
        
        % Tempo de simulação
        Ts(n) = t;
        
        % Tensões de polo de referência
        vgb10b_refs(n) = vgb10b_ref;
        vgb20b_refs(n) = vgb20b_ref;
        
        vha0a_refs(n) = vha0a_ref;
        vga0a_refs(n) = vga0a_ref;
        vla0a_refs(n) = vla0a_ref;
        
        % Tensões de referência
        vg_refs(n) = vg_ref;
        vl_refs(n) = vl_ref;
        
        vgba_refs(n) = vgba_ref;
        vgb_refs(n) = vgb_ref;
        
        % Tensões de polo
        vgb10bs(n) = vgb10b; 
        vgb20bs(n) = vgb20b;
        
        vha0as(n) = vha0a;
        vga0as(n) = vga0a;
        vla0as(n) = vla0a;
        
        % Tensões de fase
        vgs(n) = vg;
        vls(n) = vl;    
        
        % Tensões médias
        vgb10b_meds(n) = vgb10b_med;
        vgb20b_meds(n) = vgb20b_med;
        
        vha0a_meds(n) = vha0a_med;
        vga0a_meds(n) = vga0a_med;
        vla0a_meds(n) = vla0a_med;   
        
        % Corrente de refência do controlador
        ig_refs(n) = ig_ref;
        
        % Correntes
        ihs(n) = ih;
        igs(n) = ig;
        ils(n) = il;
        
        % Corrente e tensão nos barramentos
        % 2L
        ic2Ls(n) = ic2L;
        Vc2L_refs(n) = Vc2L_ref;
        Vc2Ls(n) = Vc2L;        
        
        % 3L
        ic3Ls(n) = ic3L;
        Vc3L_refs(n) = Vc3L_ref;
        Vc3Ls(n) = Vc3L;

        % Potência no barramento dos conversores
        % 2L
        Pot2Ls(n) = Pot2L;
        Pot2L_meds(n) = Pot2L_med;

        % 3L
        Pot3Ls(n)= Pot3L;
        Pot3L_meds(n) = Pot3L_med;
        
        % Portadora triangular
        vtriangles(n) = vtriangle;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Verificação da Potência por ciclo 60 Hz
        
        Pot2Lmax_totals(n) = Pot2Lmax_total;
        Pot2Lmin_totals(n) = Pot2Lmin_total;
        Pot2Lmax_total = 0;
        Pot2Lmin_total = 0;
    end
end

%%%%%%%
% Plots

figure(1) %vg
plot(Ts,vg_refs,...
     Ts,vga0a_meds-vha0a_meds+vgb10b_meds-vgb20b_meds,...
     Ts,vgs),zoom
title('Tensão vg','FontSize',18)
legend('vg_{ref}','vg_{med}','vg_{pwm}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")
axis([0 tf -210 210])

figure(2) %vl
plot(Ts,vl_refs,...
     Ts,vla0a_meds-vha0a_meds,...
     Ts,vls),zoom
title('Tensão vl','FontSize',18)
legend('vl_{ref}','vl_{med}','vl_{pwm}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")
axis([0 tf -110 110])

figure(3)
plot(Ts,igs,Ts,ils),zoom
title('Correntes','FontSize',18)
legend('ig','il','FontSize',16)
xlabel("Tempo (s)")
ylabel("Corrente (A)")

% figure(4)
% plot(Ts,vg_refs,Ts,-vgb_refs + (vga0a_refs-vha0a_refs),'r--'),zoom
% title('v*g vs Modelo','FontSize',18)
% legend('vg_{ref}','vg_{modelo}','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% axis([0 tf -110 110])

% figure(5)
% plot(Ts,vl_refs,Ts,vla0a_refs-vha0a_refs,'r--'),zoom
% title('v*l vs Modelo','FontSize',18)
% legend('vl_{ref}','vl_{modelo}','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% axis([0 tf -110 110])

% figure(6)
% plot(Ts,vga0a_refs,Ts,vha0a_refs,'k-',Ts,vla0a_refs,'b-'),zoom
% title('Tensões de polo de referência: Conversor 3L','FontSize',18)
% legend('vga0_{ref}','vha0_{ref}','val0_{ref}','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% axis([0 tf -110 110])

% figure(7)
% plot(Ts,vgb10b_refs,Ts,vgb20b_refs,'r-'),zoom
% title('Tensões de polo de referência: Conversor 2L','FontSize',18)
% legend('vgb10','vgb20','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% axis([0 tf -100 100])

figure(8)
yyaxis left
plot(Ts,igs,...
     Ts,ig_refs)
ylabel("Corrente (A)")
yyaxis right
plot(Ts,vga0a_meds-vha0a_meds+vgb10b_meds-vgb20b_meds,...
     Ts,vg_refs)
title('Corrente e Tensão da rede: medida e de referência','FontSize',18)
legend('ig_{med}','ig_{ref}','vg_{med}','vg_{ref}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")
axis([0 tf -110 110])

figure(9)
plot(Ts,Vc3Ls,Ts,Vc3L_refs,'r-'),zoom
title('Tensão do barramento 3L','FontSize',18)
legend('Vdc3L_{med}','Vdc3L_{ref}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")

figure(10)
plot(Ts,Vc2Ls,Ts,Vc2L_refs,'r-',...
    [0 tf],[Vc2L_ref+delta Vc2L_ref+delta],'k-',...
    [0 tf],[Vc2L_ref-delta Vc2L_ref-delta],'k-'),zoom
title('Tensão do barramento 2L ~ Delta = 5%Vdc2L_{ref}','FontSize',18)
legend('Vdc2L_{med}','Vdc2L_{ref}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")

figure(11)
plot(Ts,Pot2L_meds,'r-',[0 tf],[0 0],'k'),zoom
title('Potência média: barramento 2L','FontSize',18)
legend('Pot_{2L}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Potência (W)")

% figure(12) % Potência barramento 3L
% plot(Ts,Pot3L_meds,'r-',[0 tf],[0 0],'k'),zoom
% title('Potência média: barramento 3L','FontSize',18)
% legend('Pot_{2L}','FontSize',16)
% xlabel("Tempo (s)")
% ylabel("Potência (W)")

figure(13)
plot(Ts,Pot2Lmax_totals,...
     Ts,Pot2Lmin_totals,'LineWidth',2)
title('Potência de carregamento e descarregamento por ciclo de 60 Hz','FontSize',18)
legend('Potência de Carregamento','Potência de Descarregamento','FontSize',16)
xlabel("Tempo (s)")
ylabel("Potência (W)")