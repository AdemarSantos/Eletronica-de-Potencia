clear;close all;clc

% %---------------- Parâmetros do Circuito RLC-RL ------------------
E = 127;                        % Tensão do barramento CC

Vl_ref = 0.95*E;                % Amplitude da tensão de referência braço L

fg = 60;                        % Frequência da rede
fl = fg;                        % Frequência da tensão sobre a carga
w = 2*pi*fg;                    % Frequência angular

Pl = 1000;                      % Potência ativa da carga
fpl = 0.95;                     % Fator de potência da carga

Nl = Pl/fpl;                    % Potência aparente da carga
Sl = Nl*exp(j*acos(fpl));       % Potência complexa da carga
Ql = imag(Sl);                  % Potência reativa da carga

Zl = (Vl_ref^2)/conj(Sl);       % Impedância da carga
Rl = real(Zl);                  % Resistência da carga
Xl = imag(Zl);                  % Reatância da carga
Ll = Xl/(w);                    % Indutância da carga

Zb = (E^2)/Pl;                  % Impedância do barramento

Xg = 0.2*Zb;                    % Reatância da rede
Rg = 0.02*Zb;                   % Resistência da rede
Lg = Xg/(2*pi*fg);              % Indutância da rede

% Cálculo do valor eficaz da corrente drenada da rede
Ig = roots([Rg -E Pl]);
Ig = min(Ig);

faseg = (180/pi)*atan( (-Ig*Xg)/(E - Ig*Rg) );
Vg_ref = (-Ig*Xg)/(sin(faseg*pi/180));

m = 1;                          % Índice de modulação
E = (max([sqrt(2)*Vg_ref sqrt(2)*Vl_ref]))/m;

%---------------- Condições do Sistema ------------------
u = 0;                          % Definição do fator de repartição

ig = 0;                         % Corrente inicial no ramo g
il = 0;                         % Corrente inicial no ramo l

va0_int = 0;                    % valor inicial da integral da tensão no polo a
vg0_int = 0;                    % valor inicial da integral da tensão no polo g
vl0_int = 0;                    % valor inicial da integral da tensão no polo l

%---------------- Parâmetros de Simulação ------------------
h = 1E-6;                       % Passo de cálculo
t = 0;                          % Tempo inicial de simulação
tf = 60/60;

%---------------- Parâmetros de Gravação ------------------
tsave0 = 0.;                    % Tempo inicial de gravação
tsave = tsave0;                 % Tempo de gravação
npt = 50000;                    % Dimensão do vetor de saída de dados
hsave = (tf-tsave0)/npt;        % Passo de de gravação dos vetores de saída de dados

if hsave < h                    % Sendo o Passo de gravação menor que o passo de cálculo (Saturação)
    hsave = h;                  % Defina o passo de gravação = passo de cálculo
    npt = (tf-tsave0)/hsave;    % Recalcule o a dimensão dos vetores de saída de dados
end
n = 0;                          % Inicialização da variável de posição dos vetores de saída

hsave = h;

%---------------- Sinal para cálculo do valor médio ------------------
t_s = 0;
f_s = 10E3;                     % Frequência do cálculo do valor médio
h_s = 1/f_s;                    % Período do cálculo do valor médio 

tc = 0;

%---------------- Configuração dos estados das chaves ------------------
q000 = [0 0 0];
q001 = [0 0 1];
q010 = [0 1 0];
q011 = [0 1 1];
q100 = [1 0 0];
q101 = [1 0 1];
q110 = [1 1 0];
q111 = [1 1 1];

%---------------- Configuração dos estados das tensões ------------------
v000 = [0 0];           % Vg = 0, Vl = 0
v001 = [0 E];           % Vg = 0, Vl = E
v010 = [E 0];           % Vg = E, Vl = 0
v011 = [E E];           % Vg = E, Vl = E
v100 = [-E -E];         % Vg =-E, Vl =-E
v101 = [-E 0];          % Vg =-E, Vl = 0
v110 = [0 -E];          % Vg = 0, Vl =-E
v111 = [0 0];           % Vg = 0, Vl = 0

while t<tf
    t = t + h;
    
    % Onda triangular
    if t >= t_s
        t_s = t_s + h_s;
        
        % Tensões médias nos braços
        va0_med = va0_int/h_s;
        vg0_med = vg0_int/h_s;
        vl0_med = vl0_int/h_s;
        
        va0_int = 0;
        vg0_int = 0;
        vl0_int = 0;
        
        % Tensões de referência
        eg = E*sqrt(2)*cos(2*pi*60*t);      % Tensão de entrada
        vg_ref = sqrt(2)*Vg_ref*cos(2*pi*fg*t + faseg*pi/180);
        vl_ref = sqrt(2)*Vl_ref*cos(2*pi*fl*t);
        
        il1 = (sqrt(2)*(Vl_ref/abs(Zl)))*cos(2*pi*fl*t - acos(fpl) );
        
        q7 = q111;
        q0 = q000;
        
        % Definição da região onde ocorre o chaveamento
        if vg_ref > vl_ref && vg_ref > 0 && vl_ref >= 0
            k = 1;
            
            tn = (vg_ref - vl_ref)*(h_s/E); % Tempo de permanência na configuração n das chaves
            tn1 = vl_ref * (h_s/E);         % Tempo de permanência na configuração n+1 das chaves
            to = h_s - (tn + tn1);          % Tempo restante dentro do período de chaveamento
            t7 = u * to;                          % Tempo de permanência na configuração Q111 
            t0 = (1 - u)*to;                      % Tempo de permanência na configuração Q000
            
            qn  = q100;                           
            qn1 = q110;
            
        elseif vg_ref <= vl_ref && vg_ref > 0 && vl_ref > 0
            k = 2;
            
            tn = vg_ref * (h_s/E);            
            tn1 = (vl_ref - vg_ref)*(h_s/E);         
            
            to = h_s - (tn + tn1);          
            t7 = u * to;                          
            t0 = (1 - u)*to;                     
            
            qn  = q110;
            qn1 = q010;
            
        elseif vg_ref <= 0 && vl_ref > 0
            k = 3;
            
            tn = vl_ref*(h_s/E);           
            tn1 = -vg_ref*(h_s/E);
            
            to = h_s - (tn + tn1);          
            t7 = u * to;                          
            t0 = (1 - u)*to;                     
            
            qn  = q010;
            qn1 = q011;            
            
        elseif vg_ref < vl_ref && vg_ref < 0 && vl_ref <= 0   
            k = 4;
            
            tn = (vl_ref-vg_ref)*(h_s/E);           
            tn1 = -vl_ref*(h_s/E);
            
            to = h_s - (tn + tn1);          
            t7 = u * to;                          
            t0 = (1 - u)*to;
            
            qn  = q011;
            qn1 = q001; 
            
        elseif vg_ref >= vl_ref && vg_ref < 0 && vl_ref < 0   
            k = 5;
            tn = -vg_ref*(h_s/E);          
            tn1 = (vg_ref-vl_ref)*(h_s/E);
            
            to = h_s - (tn + tn1);          
            t7 = u * to;                          
            t0 = (1 - u)*to;
            
            qn  = q001;
            qn1 = q101;
            
        elseif vg_ref >= 0 && vl_ref < 0   
            k = 6;
            tn = -vl_ref*(h_s/E);          
            tn1 = vg_ref*(h_s/E);
            
            to = h_s - (tn + tn1);          
            t7 = u * to;                          
            t0 = (1 - u)*to;
            
            qn  = q101;
            qn1 = q100;            
        end
    end
            
    if tc < t0                  % Tempo da chave na configuração q111
        q = q7;
    elseif tc < t0 + tn         % Tempo da chave na configuração qn    
        q = qn;
    elseif tc < t0 + tn + tn1   % Tempo da chave na configuração qn+1
        q = qn1;
    else
        q = q0;                 % Tempo da chave na configuração q000
    end

    tc = mod(t,h_s);
    
    % Estado das chaves
    qg = q(1);
    ql = q(2);
    qa = q(3);
    
    % Tensões de polo
    va0 = (2*qa - 1)*(E/2);
    vg0 = (2*qg - 1)*(E/2);
    vl0 = (2*ql - 1)*(E/2);
    
    % Integração das tensões de polo
    va0_int = va0_int + va0*h;
    vg0_int = vg0_int + vg0*h;
    vl0_int = vl0_int + vl0*h;
    

    vg = vg0 - va0;
    vl = vl0 - va0;
    ig = ig*(1 - h*Rg/Lg) + (h/Lg)*(eg - vg); % Corrente do ramo g
    il = il*(1 - h*Rl/Ll) + (h/Ll)*vl;        % Corrente do ramo l   
    
    if tsave <= t
        tsave = tsave + hsave;
        n = n + 1;
        Ts(n) = t;
        va0s(n) = va0;
        vg0s(n) = vg0;
        vl0s(n) = vl0;
        vgs(n) = vg;
        vls(n) = vl;
        igs(n) = ig;
        ils(n) = il;
        il1s(n) = il1;
        vg_refs(n) = vg_ref;
        vl_refs(n) = vl_ref;
        va0_meds(n) = va0_med;
        vg0_meds(n) = vg0_med;
        vl0_meds(n) = vl0_med;
    end
end

figure(1)
subplot(2,1,1)
plot(Ts,vgs,Ts,vg0_meds-va0_meds,Ts,vg_refs,'r'), zoom
legend('Tensão de polo', 'Tensão média', 'Tensão de referência')
title('Braço G')
subplot(2,1,2)
plot(Ts,vls,Ts,vl0_meds-va0_meds,Ts,vl_refs,'r'), zoom
legend('Tensão de polo', 'Tensão média', 'Tensão de referência')
title('Braço L')

figure('Name','Corrente do circuito: Lado G')
plot(Ts,igs,'r-','LineWidth',1), zoom
title('Corrente do circuito: Lado G')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()

figure('Name','Corrente do circuito: Lado G')
plot(Ts,ils, Ts,il1s,'r-','LineWidth',1), zoom
title('Corrente do circuito: Lado L')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()
