% SIMULAÇÃO DA MOTOR DE INDUÇÃO ALIMENTADA PELO CONVERSOR TRIFÁSICO

clear;close all;clc;
echo off;

%---------------- Parâmetros de Simulação ------------------
h = 1E-6;                  % Passo de cálculo
t = 0;                     % Tempo inicial de simulação
tf = 1;

%---------------- Parâmetros de Gravação ------------------
tsave0 = 0.;               % Tempo inicial de gravação
tsave = tsave0;            % Tempo de gravação
hsave = h;

% npt = 5000;              % Dimensão do vetor de saída de dados
% 
% hsave = (tf-tsave0)/npt; % Passo de de gravação dos vetores de saída de dados
% 
% if hsave < h             % Sendo o Passo de gravação menor que o passo de cálculo (Saturação)
%     hsave = h;           % Defina o passo de gravação = passo de cálculo
%     npt = (tf-tsave0)/hsave; % Recalcule o a dimensão dos vetores de saída de dados
% end

n = 0;                     % Inicialização da variável de posição dos vetores de saída

%---------------- Parâmetros da máquina CA ------------------
rs = 0.39;                 % Resistência bobinas do estator
rr = 1.41;                 % Resistência bobinas do rotor
ls = 0.094;                % Indutância própria bobinas do estator
lr = 0.094;                % Indutância própria bobinas do rotor
msr = 0.091;               % Indutância mútua bobinas estator-rotor

lso = 0.1*ls;
Rg = 100;

jm = 0.04;
kf = 0.01;
cte_tempo_mec=jm/kf;
idt=1/(ls*lr-msr*msr);
p = 2;                     % Número de pares de polos
amsr = p*idt*msr;

%---------- Parâmetros e condições do Sistema -------------
V_ref = 220;               % Tensão de referência trifásica

E = V_ref*sqrt(3)*1.05;    % Tensão do barramento CC
u = 0.5 ;                  % Definição do fator de repartição

V1_ref = V_ref;
V2_ref = V_ref;
V3_ref = V_ref;            % Equilibrio de tensões

ce = 0;                    % Conjugado eletromagnético
cm = 0;                    % Conjugado mecânico
wm = 0.;                   % Velocidade da máquina

f_ref = 60;                % Frequência de referência
w_ref = 2*pi*f_ref;        % Frequência angular de referência

i1 = 0;                    % Corrente inicial no ramo 1
i2 = 0;                    % Corrente inicial no ramo 2
i3 = 0;                    % Corrente inicial no ramo 3

v10_int = 0;               % Integral da tensão no polo 1
v20_int = 0;               % Integral da tensão no polo 2
v30_int = 0;               % Integral da tensão no polo 3
vn0_int = 0;

tete = 0;                  % Ângulo dos fasores da rede

v10_ref = 0;
v20_ref = 0;
v30_ref = 0;

fsd = 0;                   % Fluxo estator fase d
fsq = 0;                   % Fluxo estator fase q
frd = 0;                   % Fluxo rotor fase d
frq = 0;                   % Fluxo rotor fase q

isd = 0;                   % Corrente do estator fase d
isq = 0;                   % Corrente do estator fase q
ird = 0;                   % Corrente do rotor fase d
irq = 0;                   % Corrente do rotor fase q

iso = 0;                   % Corrente de sequrência 0
is1 = 0;                   % Corrente da fase 1

%---------------- Inicialização da onda triangular ------------------
ttriangle = 0;             % Tempo inicial da onda triangular
ftriangle = 10E3;          % Frequência da onda triangular de 10 kHz
htriangle = 1/ftriangle;   % Período da onda triangular
vtriangle = E/2;           % Tensão máxima da onda triangular
dtriangle = E/htriangle;   % Derivada da onda triangular (?V/?T)
sign = -1;                 % Sinal inicial da derivada (Comportamento decrescente)

%--------------------------------------------------------------------
%--------------------------------------------------------------------
while t < tf
    t = t + h;
    
    % Progressão dos ângulos das tensões
	tete = tete + h*w_ref;
	if tete >= 2*pi
        tete = tete - 2*pi;
    end
    
    % Tensões da rede
    vs1 = V1_ref*cos(tete);   				
	vs2 = V2_ref*cos(tete-2*pi/3);
	vs3 = V3_ref*cos(tete+2*pi/3);
    
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
        
        % Tensões médias
        v10_med = v10_int/htriangle;
        v20_med = v20_int/htriangle;
        v30_med = v30_int/htriangle;
        vn0_med = vn0_int/htriangle;
        
        % Reset do integrador
        v10_int = 0;
        v20_int = 0;
        v30_int = 0;
        vn0_int = 0;
        
        % Sequência positiva
        v1_ref = V1_ref*cos(w_ref*t);
        v2_ref = V2_ref*cos(w_ref*t - 2*pi/3);
        v3_ref = V3_ref*cos(w_ref*t - 4*pi/3);

        % Fator de repartição
        Vn0 = [v1_ref,v2_ref,v3_ref];
        vn0max =  E/2 - max(Vn0);
        vn0min = -E/2 - min(Vn0);
        vn0_ref = u*vn0max + (1-u)*vn0min;
        
        % Tensões de polo de referência
        v10_ref = v1_ref + vn0_ref;
        v20_ref = v2_ref + vn0_ref;
        v30_ref = v3_ref + vn0_ref;

    end
    % Progressão da onda triangular
    vtriangle = vtriangle + sign*dtriangle*h;
    
    % Estado das chaves
    if v10_ref >= vtriangle % Estado das chaves (Braço 1)
        q1 = 1; % Chave fechada (IGBT em condução)
    else
        q1 = 0; % Chave aberta (IGBT em corte)
    end
    
    if v20_ref >= vtriangle % Estado das chaves (Braço 2)
        q2 = 1; % Chave fechada (IGBT em condução)
    else
        q2 = 0; % Chave aberta (IGBT em corte)
    end
    
    if v30_ref >= vtriangle % Estado das chaves (Braço 3)
        q3 = 1; % Chave fechada (IGBT em condução)
    else
        q3 = 0; % Chave aberta (IGBT em corte)
    end
    
    % Tensões de polo
    v10 = (2*q1 - 1)*(E/2);
    v20 = (2*q2 - 1)*(E/2);
    v30 = (2*q3 - 1)*(E/2);
    vn0 = (1/3)*(v10+v20+v30);
    
    % Integração das tensões de polo
    v10_int = v10_int + v10*h;
    v20_int = v20_int + v20*h;
    v30_int = v30_int + v30*h;
    vn0_int = vn0_int + vn0*h;
    
    % Tensões nos braços
    v1 = v10 - vn0;
    v2 = v20 - vn0;
    v3 = v30 - vn0;
    
    % Transformação dq
    vsd = sqrt(2/3)*(v1 - v2/2 - v3/2); 
	vsq = sqrt(2/3)*(v2*sqrt(3)/2 - v3*sqrt(3)/2);
	vso = sqrt(3)*(v1 + v2 + v3);
    
    dervfsd = vsd - rs*isd;
	dervfsq = vsq - rs*isq;
	
    dervfrd = -rr*ird - frq*wm;
	dervfrq = -rr*irq + frd*wm;
	
    deriso  = (vso - rs * iso)/lso;
    
	fsd = fsd + dervfsd*h;
	fsq = fsq + dervfsq*h;
	frd = frd + dervfrd*h;
	frq = frq + dervfrq*h;
	iso = iso + deriso*h;
    
    fso = lso*iso;
    
	ce = amsr*(fsq*frd-fsd*frq);
	
	isd = idt*(lr*fsd - msr*frd);
	isq = idt*(lr*fsq - msr*frq);
	
	ird = idt*(-msr*fsd + ls*frd);
	irq = idt*(-msr*fsq + ls*frq);
	
	is1 = sqrt(2/3)*(iso/sqrt(2) + isd);
	is2 = sqrt(2/3)*(iso/sqrt(2) - isd./2 + sqrt(3)*isq/2);
	is3 = sqrt(2/3)*(iso/sqrt(2) - isd./2 - sqrt(3)*isq/2);
	
	fs1 = sqrt(2/3)*(iso/sqrt(2) + fsd);
	fs2 = sqrt(2/3)*(iso/sqrt(2) - fsd./2 + sqrt(3)*fsq./2);
	fs3 = sqrt(2/3)*(iso/sqrt(2) - fsd./2 - sqrt(3)*fsq./2);
	
	% Equação de estado mecânica discreta
	derwm = - wm/cte_tempo_mec + p*(ce-cm)/jm;
	wm = wm + derwm*h;
    
    % Mudança súbita no conjugado mecânico
  	if t >= tf/2
          cm=40;
    end

    % Salvamento das variáveis
    if tsave <= t
        tsave = tsave + hsave;
        n = n + 1;
        
        % Tempo
        Ts(n) = t;        

        % Tensões da rede
        vs1s(n) = vs1;
        vs2s(n) = vs2;
        vs3s(n) = vs3;
        
        % Tensões de referência
        v10_refs(n) = v10_ref;
        v20_refs(n) = v20_ref;
        v30_refs(n) = v30_ref;
        
        % Tensões de fase na saída do conversor (odq)
        vsds(n) = vsd;
        vsqs(n) = vsq;
        vsos(n) = vso;
        
        % Tensões de fase na saída do conversor
        v1s(n) = v1;
        v2s(n) = v2;
        v3s(n) = v3;
        
        % Correntes de linha na saída do conversor
        is1s(n) = is1;
        is2s(n) = is2;
        is3s(n) = is3;
        
        % Fluxos
        fs1s(n) = fs1;
        fs2s(n) = fs2;
        fs3s(n) = fs3;
        
        % Tensões médias de polo
        v10_meds(n) = v10_med;
        v20_meds(n) = v20_med;
        v30_meds(n) = v30_med;
        vn0_meds(n) = vn0_med;
        
        ces(n) = ce;     % Conjugado eletromagnético 
        ves(n) = wm;     % Velocidade
        cms(n) = cm;     % Conjugado mecânico
        
        % Sinal triangular
        vtriangles(n) = vtriangle;
    end
end

% SIMULAÇÃO DO LADO 1

%---- Tensão de saída do conversor - Braço 1
figure('Name','Tensão de saída do Conversor - B1')
plot(Ts,v1s,Ts,v10_meds-vn0_meds,'r-','LineWidth',1.5),zoom
title('Tensão de saída do conversor - Braço 1','FontSize',18)
legend('vg_{pwm}','vg_{med}')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
%axis([0.1 0.2 -410 410])
grid()

%---- Corrente do circuito lado 1
figure('Name','Corrente de saída do Conversor - B1')
plot(Ts,is1s,'r-','LineWidth',1),zoom
title('Corrente de saída do conversor - Braço 1')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()

% SIMULAÇÃO DO LADO 2

%---- Tensão de saída do conversor - Braço 2
figure('Name','Tensão de saída do Conversor - B2')
plot(Ts,v2s,Ts,v20_meds-vn0_meds,'r-','LineWidth',1.5),zoom
title('Tensão de saída do Conversor - Braço 2','FontSize',18)
legend('vg_{pwm}','vg_{med}')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
%axis([0.1 0.20 -410 410])
grid()

%---- Corrente do circuito lado 2
figure('Name','Corrente de saída do conversor - B2')
plot(Ts,is2s,'r-','LineWidth',1),zoom
title('Corrente de saída do conversor - Braço 2')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()

% SIMULAÇÃO DO LADO 3

%---- Tensão de saída do conversor - Braço 3
figure('Name','Tensão de saída do conversor - B3')
plot(Ts,v3s,Ts,v30_meds-vn0_meds,'r-','LineWidth',1.5),zoom
title('Tensão de saída do conversor - Braço 3','FontSize',18)
legend('vg_{pwm}','vg_{med}')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
%axis([0.1 0.20 -410 410])
grid()

%---- Corrente do circuito lado 3
figure('Name','Corrente de saída do conversor - B3')
plot(Ts,is3s,'r-','LineWidth',1),zoom
title('Corrente de saída do conversor - Braço 3')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()

%---- Tensões
figure('Name','Tensões de saída do conversor')
plot(Ts,v10_meds-vn0_meds,Ts,v20_meds-vn0_meds,Ts,v30_meds-vn0_meds,'r-','LineWidth',1),zoom
title('Tensões médias na saída do conversor')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid()

%---- Correntes
figure('Name','Correntes de saída do conversor')
plot(Ts,is1s,Ts,is2s,Ts,is3s,'r-','LineWidth',1),zoom
title('Correntes de saída do conversor')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()

%---- Fluxos
figure('Name','Fluxos de saída do conversor')
plot(Ts,fs1s,Ts,fs2s,Ts,fs3s,'r-','LineWidth',1),zoom
title('Fluxos de saída do conversor')
xlabel("Tempo (s)")
ylabel("Fluxo (Wb)")
grid()

%---- Tensão de sequência 0 
figure('Name','Tensão de sequência 0')
plot(Ts,vsos,'r-','LineWidth',1),zoom
title('Tensão de sequência 0')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid()

%---- Velocidade
figure('Name','Velocidade')
plot(Ts,ves,'r-','LineWidth',1),zoom
title('Velocidade')
xlabel("Tempo (s)")
ylabel("Velocidade (rad/s)")
grid()

%---- Conjugado mecânico 
figure('Name','Conjugado mecânico')
plot(Ts,cms,'r-','LineWidth',1),zoom
title('Conjugado mecânico')
xlabel("Tempo (s)")
ylabel("Torque (N.m)")
grid()

%---- Conjugado eletromagnético 
figure('Name','Conjugado eletromagnético')
plot(Ts,ces,'r-','LineWidth',1),zoom
title('Conjugado eletromagnético')
xlabel("Tempo (s)")
ylabel("Torque (N.m)")
grid()