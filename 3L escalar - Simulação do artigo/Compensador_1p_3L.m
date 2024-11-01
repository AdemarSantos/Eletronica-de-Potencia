clear;close all;clc

%---------------- Parâmetros do Circuito RLC-RL ------------------
Rg = 0.1;                 % Resistência de entrada
Lg = 5e-3;                % Indutância de entrada

Rl = 30.9760;             % Resistência da carga
Ll = 0.0616;              % Indutância da carga

%---------------- Parâmetros de Simulação ------------------
h = 1E-6;                % Passo de cálculo
t = 0;                   % Tempo inicial de simulação
tf = 0.2;

%---------------- Parâmetros de Gravação ------------------
tsave0 = 0.;             % Tempo inicial de gravação
tsave = tsave0;          % Tempo de gravação
npt = 20000;             % Dimensão do vetor de saída de dados

hsave = (tf-tsave0)/npt; % Passo de de gravação dos vetores de saída de dados

if hsave < h             % Sendo o Passo de gravação menor que o passo de cálculo (Saturação)
    hsave = h;           % Defina o passo de gravação = passo de cálculo
    npt = (tf-tsave0)/hsave; % Recalcule o a dimensão dos vetores de saída de dados
end

n = 0;                   % Inicialização da variável de posição dos vetores de saída

%---------------- Condições do Sistema ------------------
E = 340;                 % Tensão do barramento CC
u = 0.5 ;                % Definição do fator de repartição

Vg_ref = 220*sqrt(2);          % Amplitude da tensão de referência braço G
Vl_ref = 220*sqrt(2);         % Amplitude da tensão de referência braço L

Eg = 220*sqrt(2);
thetal_ref = 0;       % Fase da tensão de referência braço L

f_ref = 60;              % Frequência de referência
w_ref = 2*pi*f_ref;      % Frequência angular de referência

ig = 0;                  % Corrente inicial no ramo g
il = 0;                  % Corrente inicial no ramo l

va0_int = 0;             % valor inicial da integral da tensão no polo a
vg0_int = 0;             % valor inicial da integral da tensão no polo g
vl0_int = 0;             % valor inicial da integral da tensão no polo l

%---------------- Inicialização da onda triangular ------------------
ttriangle = 0;           % Tempo inicial da onda triangular
ftriangle = 10E3;        % Frequência da onda triangular de 10 kHz
htriangle = 1/ftriangle; % Período da onda triangular
vtriangle = E/2;         % Tensão máxima da onda triangular
dtriangle = E/htriangle; % Derivada da onda triangular (?V/?T)
sign = -1;               % Sinal inicial da derivada (Comportamento decrescente)

%% Início do Looping
while t<tf
    t = t + h;
    
    % Onda triangular
    if t >= ttriangle
        ttriangle = ttriangle + htriangle;
        
        if vtriangle <= 0
           vtriangle = -E/2;
           sign = 1;
        else
            vtriangle = E/2;
            sign = -1;
        end
        
        % Tensões de referência
        vg_ref = Vg_ref*cos(w_ref*t);
        vl_ref = Vl_ref*cos(w_ref*t + thetal_ref);
        
        % Fator de repartição
        VS_ref = [vg_ref,vl_ref,0];
        vu_ref_max =  E/2 - max(VS_ref);
        vu_ref_min = -E/2 - min(VS_ref);
     
        vu_ref = u*vu_ref_max + (1 - u)*vu_ref_min;

        % Tensões de polo de referência
        va0_ref = vu_ref;
        vg0_ref = vg_ref + vu_ref;
        vl0_ref = vl_ref + vu_ref;
        
        % Tensões médias nos braços
        va0_med = va0_int/htriangle;
        vg0_med = vg0_int/htriangle;
        vl0_med = vl0_int/htriangle;
        
        va0_int = 0;
        vg0_int = 0;
        vl0_int = 0;
    end
    
    vtriangle = vtriangle + sign*dtriangle*h;
   
    % Estado das chaves
    if va0_ref >= vtriangle % Estado das chaves (Braço a)
        qa = 1; % Chave fechada
    else
        qa = 0; % Chave aberta
    end
    
    if vg0_ref >= vtriangle % Estado das chaves (Braço g)
        qg = 1; % Chave fechada
    else
        qg = 0; % Chave aberta
    end
    
    if vl0_ref >= vtriangle % Estado das chaves (Braço l)
        ql = 1; % Chave fechada (IGBT em condução)
    else
        ql = 0; % Chave aberta (IGBT em corte)
    end
    
    % Tensões de polo
    va0 = (2*qa - 1)*(E/2);
    vg0 = (2*qg - 1)*(E/2);
    vl0 = (2*ql - 1)*(E/2);
    
    % Tensão de entrada (Rede)
    eg = Eg*cos(2*pi*60*t);                    
    
    % Tensões geradas nos braços
    vg = vg0 - va0;
    vl = vl0 - va0;
    
    % Integração das tensões de polo
    va0_int = va0_int + va0*h;
    vg0_int = vg0_int + vg0*h;
    vl0_int = vl0_int + vl0*h;
    
    % Integração numérica
    ig = ig*(1 - h*Rg/Lg) + (h/Lg)*(eg - vg); % Corrente do ramo g
    il = il*(1 - h*Rl/Ll) + (h/Ll)*vl;        % Corrente do ramo l   
    ia = ig - il;
    
    % Controle do barramento
    % 3L
    Rp3L = 15e3;
    Rs3L = 0.2;
    ik3L = ig*qga - il*qla - is*qsa;
    ic3L = (Rp3L*ik3L - Vc3L)/(Rp3L + Rs3L);   % Corrente de entrada barramento 3L
    Vc3L = Vc3L + (h/Cap)*ic3L;                % Tensão barramento 3L
%     Vc3L = Vc3L_ref*1.00;  
    
    if tsave <= t
        tsave = tsave + hsave;
        n = n + 1;
        Ts(n) = t;
        
        % Tensão da rede e barramento
        egs(n) = eg;
        Es(n) = E;
        
        % Tensões de referência
        vg_refs(n) = vg_ref;
        vl_refs(n) = vl_ref;
        
        % Tensões de polo de referência
        va0_refs(n) = va0_ref;
        vg0_refs(n) = vg0_ref;
        vl0_refs(n) = vl0_ref;       
        
        % Tensões de polo
        va0s(n) = va0;
        vg0s(n) = vg0;
        vl0s(n) = vl0;
        
        % Tensões de polo médias
        va0_meds(n) = va0_med;
        vg0_meds(n) = vg0_med;
        vl0_meds(n) = vl0_med;
        
        % Tensões geradas
        vgs(n) = vg;
        vls(n) = vl;
        
        %Correntes
        igs(n) = ig;
        ils(n) = il;
        ias(n) = ia;
        
        % Estado das chaves
        qgs(n) = qg;
        qls(n) = ql;
        qas(n) = qa;
        
        % Onda triangular
        vtriangles(n) = vtriangle;
        
        % Barramento
        Vc3Ls(n) = Vc3L;
        Vc3L_refs(n) = Vc3L_ref;
        
    end
end

figure('Name','Tensão da rede')
plot(Ts,egs,Ts,Es,'r','LineWidth',1.5),zoom
title('Tensão da rede e barramento','FontSize',18)
legend('eg','E_{barramento}')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid minor

figure('Name','Tensão do Conversor Lado G')
plot(Ts,vgs,Ts,vg0_meds-va0_meds,'r','LineWidth',1.5),zoom
title('Tensão do Conversor Lado G','FontSize',18)
legend('vg_{pwm}','vg_{med}')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid minor

figure('Name','Tensão do Conversor Lado L')
plot(Ts,vls,Ts,vl0_meds-va0_meds,'r','LineWidth',1.5),zoom
title('Tensão do Conversor Lado L','FontSize',18)
legend('vl_{pwm}','vl_{med}')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid minor

figure('Name','Correntes')
plot(Ts,ils,Ts,igs,Ts,ias,'r-','LineWidth',1)
title('Correntes do Circuito')
legend('il','ig','ia')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid minor


%---- Tensão do sinal triangular
figure('Name','Sinal triangular')
plot(Ts,vtriangles,'r-','LineWidth',2)
title('Sinal triangular (Portadora)')
xlabel("Tempo (s)")
ylabel("Tensão (A)")
grid minor

%---- Tensão do sinal triangular
figure('Name','Sinal triangular ref chave g')
plot(Ts, vtriangles, Ts, vg0_refs, Ts ,qgs.*100,'r-','LineWidth',2)
title('Triangular, Referência e chave (Braço g)')
xlabel("Tempo (s)")
ylabel("Tensão (A)")
grid minor

figure('Name','Sinal triangular ref chave l')
plot(Ts, vtriangles, Ts, vl0_refs, Ts ,qls.*100,'r-','LineWidth',2)
title('Triangular, Referência e chave (Braço l)')
xlabel("Tempo (s)")
ylabel("Tensão (A)")
grid minor

figure('Name','Sinal triangular ref chave a')
plot(Ts, vtriangles, Ts, va0_refs, Ts ,qas.*100,'r-','LineWidth',2)
title('Triangular, Referência e chave (Braço a)')
xlabel("Tempo (s)")
ylabel("Tensão (A)")
grid minor