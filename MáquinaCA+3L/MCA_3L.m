clear;close all;clc
echo off

%---------------- Parâmetros de Simulação ------------------
%1.e-4; 2.e-4; 5.e-4;
h = 1E-6;                % Passo de cálculo
t = 0;                   % Tempo inicial de simulação
tf = 1;

%---------------- Parâmetros de Gravação ------------------
tsave0 = 0.;             % Tempo inicial de gravação
tsave = tsave0;          % Tempo de gravação
hsave = h;

% npt = 5000;              % Dimensão do vetor de saída de dados
% 
% hsave = (tf-tsave0)/npt; % Passo de de gravação dos vetores de saída de dados
% 
% if hsave < h             % Sendo o Passo de gravação menor que o passo de cálculo (Saturação)
%     hsave = h;           % Defina o passo de gravação = passo de cálculo
%     npt = (tf-tsave0)/hsave; % Recalcule o a dimensão dos vetores de saída de dados
% end

n = 0;                   % Inicialização da variável de posição dos vetores de saída
%---------------- Parâmetros da máquina CA ------------------
rs=0.39;                 % Resistência bobinas do estator
rr=1.41;                 % Resistência bobinas do rotor
ls=0.094;                % Indutância própria bobinas do estator
lr=0.094;                % Indutância própria bobinas do rotor
lso = 0.094;
msr=0.091;               % Indutância mútua bobinas estator-rotor

jm = 0.04; 
kf = 0.01;
cte_tempo_mec=jm/kf;
idt=1/(ls*lr-msr*msr);
p = 2;                   % Número de pares de polos
amsr = p*idt*msr;

Rg = 100;                % Resistência a ser colocada na fase d


%---------------- Condições do Sistema ------------------
E = 750;                 % Tensão do barramento CC
u = 0.5 ;                % Definição do fator de repartição

cm=0;                    % Conjugado mecânico
wm=0.;                   % Velocidade da máquina
ce = 0;                  % Conjugado eletromagnético

ws=2*pi*60;              % Frequência angular da rede
Vs=220*sqrt(2);          % Amplitude dos fasores da rede   
tete=0;                  % Ângulo dos fasores da rede

Vg_ref = 1.1*220*sqrt(2);    % Amplitude da tensão de referência braço G
Vl_ref = 1.1*220*sqrt(2);    % Amplitude da tensão de referência braço L

f_ref = 60;              % Frequência de referência
w_ref = 2*pi*f_ref;      % Frequência angular de referência

ig = 0;                  % Corrente inicial no ramo g
il = 0;                  % Corrente inicial no ramo l

fsd=0;                   % Fluxo do estator fase d
fsq=0;                   % Fluxo do estator fase q
fso=0;

frd=0;                   % Fluxo do rotor fase d
frq=0;                   % Fluxo do rotor fase q
fro=0;

isd=0;                   % Corrente do estator fase d
isq=0;                   % Corrente do estator fase q

ird=0;                   % Corrente do rotor fase d
irq=0;                   % Corrente do rotor fase q

va0_int = 0;             % Integral da tensão no polo a
vg0_int = 0;             % Integral da tensão no polo g
vl0_int = 0;             % Integral da tensão no polo l

%---------------- Inicialização da onda triangular ------------------
ttriangle = 0;           % Tempo inicial da onda triangular
ftriangle = 10E3;        % Frequência da onda triangular de 10 kHz
htriangle = 1/ftriangle; % Período da onda triangular
vtriangle = E/2;         % Tensão máxima da onda triangular
dtriangle = E/htriangle; % Derivada da onda triangular (?V/?T)
sign = -1;               % Sinal inicial da derivada (Comportamento decrescente)

while t<tf
    t = t + h;
    
    % Progressão dos ângulos das tensões
    tete = tete + h*ws;					
    if tete >= 2*pi
        tete = tete - 2*pi;
    end
    
    % Tensões da rede
    vs1=Vs*cos(tete);   				
	vs2=Vs*cos(tete-2*pi/3);
	vs3=Vs*cos(tete+2*pi/3);
    
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
        
        % Tensões médias nos braços
        va0_med = va0_int/htriangle;
        vg0_med = vg0_int/htriangle;
        vl0_med = vl0_int/htriangle;
        
        va0_int = 0;
        vg0_int = 0;
        vl0_int = 0;
        
        % Tensões de referência
        vg_ref = Vg_ref*cos(w_ref*t);
        vl_ref = Vl_ref*sin(w_ref*t);
        
        % Fator de repartição
        VS_ref = [vg_ref,vl_ref,0];
        vu_ref_max =  max(VS_ref);
        vu_ref_min =  min(VS_ref);
        vu_ref = E*(u - 1/2) - u*vu_ref_max + (u - 1)*vu_ref_min;
        
        % Tensões de polo de referência
%         va0_ref = vu_ref;
%         vg0_ref = vg_ref + vu_ref;
%         vl0_ref = vl_ref + vu_ref;
        vg0_ref = (Vg_ref)*cos(w_ref*t);
        va0_ref = (Vg_ref)*cos(w_ref*t+2*pi/3);
        vl0_ref = (Vg_ref)*cos(w_ref*t-2*pi/3);
    end
    % Progressão da onda triangular
    vtriangle = vtriangle + sign*dtriangle*h;
    
    % Estado das chaves
    if va0_ref >= vtriangle % Estado das chaves (Braço a)
        qa = 1; % Chave fechada (IGBT em condução)
    else
        qa = 0; % Chave aberta (IGBT em corte)
    end
    
    if vg0_ref >= vtriangle % Estado das chaves (Braço g)
        qg = 1; % Chave fechada (IGBT em condução)
    else
        qg = 0; % Chave aberta (IGBT em corte)
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
    
    % Integração das tensões de polo
    va0_int = va0_int + va0*h;
    vg0_int = vg0_int + vg0*h;
    vl0_int = vl0_int + vl0*h;
    
    % Tensões nos braços
    vg = vg0 - va0;
    vl = vl0 - va0;
    
    % Abertura da fase d
    if t >= tf/2 
        vsd = vg - Rg*isd; 
    else
        vsd = vg;
    end
    vsq = vl;
    
    % Derivadas dos fluxos
    dervfsd = vsd - rs*isd;
	dervfsq = vsq - rs*isq;

    
	dervfrd = -rr*ird - frq*wm;
	dervfrq = -rr*irq + frd*wm;

    
    % Fluxos
% 	fsd = fsd + dervfsd*h;
% 	fsq = fsq + dervfsq*h;
% 	fso = fso + dervfso*h;
%     
%     frd = frd + dervfrd*h;
% 	frq = frq + dervfrq*h;
    
    fsd = fsd + dervfsd*h;
	fsq = fsq + dervfsq*h;
	frd = frd + dervfrd*h;
	frq = frq + dervfrq*h;
    
    % Conjugado (Produto vetorial entre os vetores de fluxo girantes)
    ce = amsr*(fsq*frd-fsd*frq);
	
    % Correntes do estator
	isd=idt*(lr*fsd - msr*frd);
	isq=idt*(lr*fsq - msr*frq);
	
    % Correntes do rotor
	ird=idt*(-msr*fsd + ls*frd);
	irq=idt*(-msr*fsq + ls*frq);
	
    % Representação trifásica das correntes
	is1=sqrt(2/3)*isd;
	is2=sqrt(2/3)*(-isd./2 + sqrt(3)*isq/2);
	is3=sqrt(2/3)*(-isd./2 - sqrt(3)*isq/2);
	
    % Representação trifásica dos fluxos
	fs1=sqrt(2/3)*fsd;
	fs2=sqrt(2/3)*(-fsd./2 + sqrt(3)*fsq./2);
	fs3=sqrt(2/3)*(-fsd./2 - sqrt(3)*fsq./2);
    
    % Equacao de estado mecânica discreta
	derwm = - wm/cte_tempo_mec + p*(ce-cm)/jm;
	wm = wm + derwm*h;       
	
    % Mudança no Conjugado mecânico
    %if t >= tf/2
    %   cm = 40;
    %end
    
    % Salvamento das variáveis
    if tsave <= t
        tsave = tsave + hsave;
        n = n + 1;
        
        Ts(n) = t;        
        corrented(n) = isd;
        correnteq(n) = isq;
        corrente1(n) = is1;
        corrente2(n) = is2;
        corrente3(n) = is3;
        tensao1(n) = vs1;
        tensao2(n) = vs2;
        tensao3(n) = vs3;
        tensaosd(n) = vsd;
        tensaosq(n) = vsq;
        fluxord(n) = frd;
        fluxorq(n) = frq;
        fluxos1(n) = fs1;
        fluxos2(n) = fs2;
        fluxos3(n) = fs3;
        fluxosd(n) = fsd;
        fluxosq(n) = fsq;
        conjugado(n) = ce;
        velocidade(n) = wm;
        frequencia(n) = ws;
        conjcarga(n) = cm;
        va0s(n) = va0;
        vg0s(n) = vg0;
        vl0s(n) = vl0;
        vgs(n) = vg;
        vls(n) = vl;
        igs(n) = isd;
        ils(n) = isq;
        va0_meds(n) = va0_med;
        vg0_meds(n) = vg0_med;
        vl0_meds(n) = vl0_med;
        va0_refs(n) = va0_ref;
        vg0_refs(n) = vg0_ref;
        vl0_refs(n) = vl0_ref;
        vtriangles(n) = vtriangle;
    end
end

% SIMULAÇÃO DO LADO G
%---- Saída do conversor G
figure(1),plot(Ts,corrente1, Ts,corrente2, Ts,corrente3),zoom
title('Correntes');


figure('Name','Tensão de saída do Conversor G')
plot(Ts,vgs,Ts,vg0_meds-va0_meds,'r-','LineWidth',1.5),zoom
title('Tensão de saída do conversor G','FontSize',18)
legend('vg_{pwm}','vg_{med}')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
%axis([0.1 0.2 -410 410])
grid()

%---- Corrente do circuito lado G
figure('Name','Corrente do circuito: Lado G')
plot(Ts,corrented,'r-','LineWidth',1),zoom
title('Corrente do circuito: Lado G')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()

% SIMULAÇÃO DO LADO L
%---- Saída do conversor L

figure('Name','Tensão de saída do Conversor G')
plot(Ts,vls,Ts,vl0_meds-va0_meds,'r-','LineWidth',1.5),zoom
title('Tensão de saída do conversor L','FontSize',18)
legend('vg_{pwm}','vg_{med}')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
%axis([0.1 0.20 -410 410])
grid()

%---- Corrente do circuito lado L
figure('Name','Corrente do circuito: Lado G')
plot(Ts,correnteq,'r-','LineWidth',1),zoom
title('Corrente do circuito: Lado L')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()

%---- Saída do conversor

figure('Name','Tensão de saída')
subplot(3,1,1)
plot(Ts,vl0s,'k-',Ts,vl0_meds,'r-','LineWidth',1.5),zoom
legend('v_{1_{pwm}}','v_{1_{med}}')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
axis([0.1 0.13 -410 410])
grid minor
subplot(3,1,2)
plot(Ts,va0s,'k-',Ts,va0_meds,'-','LineWidth',1.5),zoom
legend('v_{2_{pwm}}','v_{2_{med}}')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
axis([0.1 0.13 -410 410])
grid minor
subplot(3,1,3)
plot(Ts,vg0s,'k-',Ts,vg0_meds,'y-','LineWidth',1.5),zoom
legend('v_{3_{pwm}}','v_{3_{med}}')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
axis([0.1 0.13 -410 410])
grid minor