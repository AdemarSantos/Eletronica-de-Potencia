% Conversor 3L (2L série + 1L shunt)

clear;close all;clc

%---------------- Condições do Sistema ------------------
E = 340;                 % Tensão do barramento CC

Vl_ref = 220*sqrt(2);    % Amplitude da tensão de referência braço L
Vg_ref = 150*sqrt(2);
Vgm = 150*sqrt(2);       % Amplitude da tensão da rede

thetal_ref = 0;          % Fase da tensão de referência braço L

f_ref = 60;              % Frequência de referência
w_ref = 2*pi*f_ref;      % Frequência angular de referência

Ig = 0;
Cap = 4.4E-3;            % Capacitância do barramento

ig = 0;                  % Corrente inicial no ramo g
il = 0;                  % Corrente inicial no ramo l
ih = 0;                  % Corrente inicial no ramo s

vh0_int = 0;             % valor inicial da integral da tensão no polo a
vg0_int = 0;             % valor inicial da integral da tensão no polo g
vl0_int = 0;             % valor inicial da integral da tensão no polo l
el_int = 0;

%---------------- Impedâncias de entrada e saída ------------------
Pl = 1000;               % Potência ativa consumida na carga [W]
FPl = 0.80; % Ind.        % Fator de potência na carga
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
Lg = 5e-3;               % Indutância
Xg = Lg*w_ref;           % Reatância
 
FPg = 1;                 % Fator de potência da rede
thetag = acos(FPg);      % Defasagem tensão-corrente da rede

%---------------- Parâmetros de Simulação ------------------
h = 1E-7;                % Passo de cálculo
t = 0;                   % Tempo inicial de simulação
tf = 2;

%---------------- Parâmetros de Gravação ------------------
tsave0 = 0.;             % Tempo inicial de gravação
tsave = tsave0;          % Tempo de gravação
npt = 100000;            % Dimensão do vetor de saída de dados

hsave = (tf-tsave0)/npt; % Passo de de gravação dos vetores de saída de dados
hsave = 1E-6;

if hsave < h             % Sendo o Passo de gravação menor que o passo de cálculo (Saturação)
    hsave = h;           % Defina o passo de gravação = passo de cálculo
    npt = (tf-tsave0)/hsave; % Recalcule o a dimensão dos vetores de saída de dados
end
n = 0;                   % Inicialização da variável de posição dos vetores de saída

%---------------- Parâmetros do controlador ------------------
Kp = 0.2;
Ki = 15;

P_error = 0;
I_error = 0;

vc_ref = E;
vc = E;

Ih0 = 3;

%---------------- Inicialização da onda triangular ------------------
ttri = 0;           % Tempo inicial da onda triangular
ftri = 10E3;        % Frequência da onda triangular de 10 kHz
htri = 1/ftri; % Período da onda triangular
vtri = 1/2;         % Tensão máxima da onda triangular
dtri = 1/htri; % Derivada da onda triangular (?V/?T)
sign = -1;               % Sinal inicial da derivada (Comportamento decrescente)

while t<tf
    t = t + h;
    
%     if t >= tf/2
%         Vgm = 300*sqrt(2);
%     end
    
    eg = Vgm*cos(w_ref*t);
    
    % Onda triangular
    if t >= ttri
        ttri = ttri + htri;
        
        % Controlador PI da corrente ig
        vc_error = vc_ref - vc;
        P_error = Kp*vc_error;
        I_error = I_error + htri*Ki*vc_error;
        Ig = P_error + I_error + Ih0;
        
        ig_ref = Ig*cos(w_ref*t - thetag);
        ig_error = (ig_ref - ig);
        
        % Tensões de referência
        vg_ref = eg - Rg*ig - (Lg/htri)*ig_error;
        vl_ref = eg - Vl_ref*cos(w_ref*t + thetal_ref);        

        % Fator de repartição
        VS_ref = [vg_ref,vl_ref,0];
        vu_ref_max =  E/2 + max(VS_ref);
        vu_ref_min = -E/2 + min(VS_ref);
        vu_ref = (vu_ref_max+vu_ref_min)/2;
        
        % Tensões de polo de referência
        % vg = vg0 - vh0;
        % vl = vg0 - vl0;
        
        vg0_ref = vu_ref;
        vh0_ref = vu_ref - vg_ref;
        vl0_ref = vu_ref - vl_ref;        
        
        % Tensões médias nos braços
        vh0_med = vh0_int/htri;
        vg0_med = vg0_int/htri;
        vl0_med = vl0_int/htri;
        el_med = el_int/htri;
        
        vh0_int = 0;
        vg0_int = 0;
        vl0_int = 0;        
        el_int = 0;
        
        % Onda triangular
        if vtri <= 0
           vtri = -(1/2);
           sign = 1;
        else
            vtri = (1/2);
            sign = -1;
        end
        
    end
    
    % Progressão da portadora triangular
    vtri = vtri + sign*dtri*h;
    vt = vtri*E;
    
    % Estado das chaves
    if vh0_ref >= vt % Estado das chaves (Braço h)
        qh = 1; % Chave fechada
    else
        qh = 0; % Chave aberta
    end
    
    if vg0_ref >= vt % Estado das chaves (Braço g)
        qg = 1; % Chave fechada
    else
        qg = 0; % Chave aberta
    end
    
    if vl0_ref >= vt % Estado das chaves (Braço l)
        ql = 1; % Chave fechada (IGBT em condução)
    else
        ql = 0; % Chave aberta (IGBT em corte)
    end
    
    % Tensões de polo
    vh0 = (2*qh - 1)*(E/2);
    vg0 = (2*qg - 1)*(E/2);
    vl0 = (2*ql - 1)*(E/2);
    
    % Tensões geradas
    vg = vg0 - vh0;
    vl = vg0 - vl0;
    el = eg - vl;
    
    % Integração das tensões de polo
    vh0_int = vh0_int + vh0*h;
    vg0_int = vg0_int + vg0*h;
    vl0_int = vl0_int + vl0*h;    
    el_int = el_int + el*h;
    
    % Integração numérica das correntes ig e il
    il = il*(1 - h*Rl/Ll) + (h/Ll)*(eg - vl);
    ih = ih*(1 - h*Rg/Lg) + (h/Lg)*(eg - vg);
    ig = ih + il;
    
    % Controle do barramento
    ic = ig*qg - il*ql - ih*qh;
    vc = vc + h*(ic/Cap);
%     vc = vc_ref;
    
    if tsave <= t
        tsave = tsave + hsave;
        n = n + 1;
        Ts(n) = t;
        
        % Estado das chaves
        qhs(n) = qh;
        qgs(n) = qg;
        qls(n) = ql;
        
        % Tensões de polo de referência
        vh0_refs(n) = vh0_ref;
        vg0_refs(n) = vg0_ref;
        vl0_refs(n) = vl0_ref;
        
        % Tensões de referência 
        vg_refs(n) = vg_ref;
        vl_refs(n) = vl_ref;  
        
        % Tensões de polo
        vh0s(n) = vh0;
        vg0s(n) = vg0;
        vl0s(n) = vl0;
        
        % Tensões geradas
        vgs(n) = vg;
        vls(n) = vl;    
        els(n) = el; % Carga
        
        % Tensões médias
        vh0_meds(n) = vh0_med;
        vg0_meds(n) = vg0_med;
        vl0_meds(n) = vl0_med;   
        el_meds(n) = el_med; % Carga
        
        % Corrente de referência do controlador
        ig_refs(n) = ig_ref;
        
        % Correntes
        ihs(n) = ih;
        igs(n) = ig;
        ils(n) = il;
        
        % Barramento
        vcs(n) = vc;
        vc_refs(n) = vc_ref;
        
        % Portadora triangular
        vtris(n) = vtri;
        vts(n) = vt;
    end
end

%---- Tensões Vg e Vl
figure('name','Tensão Vg') %vg
plot(Ts,vg_refs,...
     Ts,vg0_meds-vh0_meds,...
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

%---- Correntes
figure('Name','Corrente do circuito: Lado G, A e L')
plot(Ts,igs,Ts,ils,Ts,ihs,'r-','LineWidth',1),zoom
title('Corrente do circuito: ig, il e ia')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
legend('ig','il','ih','FontSize',18)
grid()

%---- Tensão barramento
figure('name','Tensão no barramento 3L')
plot(Ts,vcs,Ts,vc_refs,'r--'),zoom
title('Tensão do barramento 3L','FontSize',18)
legend('Vdc3L_{med}','Vdc3L_{ref}','FontSize',16)
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid('minor')