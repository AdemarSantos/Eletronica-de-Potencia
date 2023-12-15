% SIMULACAO DO MOTOR DE INDUÇÃO

clear all;close all; clc;
echo off;

%---------------- Constantes ------------------
pi23 = 2*pi/3;
rq23 = sqrt(2/3);
rq3 = sqrt(3);

%---------------- Parâmetros de Simulação -------------------
h = 1.e-4;                 % 1.e-4; 2.e-4; 5.e-4;
t = 0;                     % Tempo inicial de simulação
tf = 1;                    % Tempo final de simulação

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

%---------------- Parâmetros de Gravação ------------------
tsave0 = 0.;             % Tempo inicial de gravação
tsave = tsave0;          % Tempo de gravação
hsave = h;               % Passo de de gravação dos vetores de saída de dados

% npt = 5000;              % Dimensão do vetor de saída de dados
% 
% hsave = (tf-tsave0)/npt; % Passo de de gravação dos vetores de saída de dados
% 
% if hsave < h             % Sendo o Passo de gravação menor que o passo de cálculo (Saturação)
%     hsave = h;           % Defina o passo de gravação = passo de cálculo
%     npt = (tf-tsave0)/hsave; % Recalcule o a dimensão dos vetores de saída de dados
% end

n = 0;                   % Inicialização da variável de posição dos vetores de saída

%---------------- Condições do Sistema ------------------
ce = 0;                  % Conjugado eletromagnético
cm=0;                    % Conjugado mecânico
wm=0.;                   % Velocidade da máquina

ws=2*pi*60;              % Frequência angular da rede
Vs=220*sqrt(2);          % Amplitude dos fasores da rede   
tete=0;                  % Ângulo dos fasores da rede

fsd = 0;                 % Fluxo estator fase d
fsq = 0;                 % Fluxo estator fase q
frd = 0;                 % Fluxo rotor fase d
frq = 0;                 % Fluxo rotor fase q

isd = 0;                 % Corrente do estator fase d
isq = 0;                 % Corrente do estator fase q
ird = 0;                 % Corrente do rotor fase d
irq = 0;                 % Corrente do rotor fase q

iso = 0;                 % Corrente de sequrência 0
is1 = 0;                 % Corrente da fase 1


while t <= tf
	t = t + h;
	
	tete = tete + h*ws;						% Ângulo da tensão
	
    if tete >= 2*pi
        tete = tete - 2*pi;
    end
    % Tensões da rede
    vs2 = Vs*cos(tete-pi23);
	vs3 = Vs*cos(tete+pi23);
    vs1 = Vs*cos(tete);
    
    % Abertura súbita de uma das fases
%     if t >= tf/2
%         vs1 = Vs*cos(tete) - Rg*is1;   				
%     else
%         vs1 = Vs*cos(tete);
%     end
    
	% Máquina de Indução
	%---------------------------------------
	%---------------------------------------
	
    vsd = rq23.*(vs1 - vs2./2 - vs3./2); 
	vsq = rq23.*(vs2*rq3./2 - vs3*rq3./2);
	vso = (rq23/sqrt(2)).*(vs1 + vs2 +vs3);
    
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
	
	is1=rq23*(iso/sqrt(2) + isd);
	is2=rq23*(iso/sqrt(2) - isd./2 + rq3*isq/2);
	is3=rq23*(iso/sqrt(2) - isd./2 - rq3*isq/2);
	
	fs1=rq23*(iso/sqrt(2) + fsd);
	fs2=rq23*(iso/sqrt(2) - fsd./2 + rq3*fsq./2);
	fs3=rq23*(iso/sqrt(2) - fsd./2 - rq3*fsq./2);
	
	% Equação de estado mecânica discreta
	derwm = - wm/cte_tempo_mec + p*(ce-cm)/jm;
	wm = wm + derwm*h;       
	
    % Mudança súbita no conjugado mecânico
% 	if t >= tf/2
%         cm=40;
%   end

    % Armazenagem das variáveis de saída
	if tsave <= t 
        tsave = tsave + hsave;        
        n = n + 1;
        
        % Tempo
        Ts(n) = t;
        
        % Correntes
        isds(n) = isd;   % Corrente estator fase d
        isqs(n) = isq;   % Corrente estator fase q
        isos(n) = iso;   % Corrente de sequência 0
        is1s(n) = is1;   % Corrente fase 1
        is2s(n) = is2;   % Corrente fase 2
        is3s(n) = is3;   % Corrente fase 3
        
        % Tensões
        vsds(n) = vsd;   % Tensão estator fase d
        vsqs(n) = vsq;   % Tensão estator fase q
        vsos(n) = vso;   % Tensão de sequência 0
        vs1s(n) = vs1;   % Tensão fase 1
        vs2s(n) = vs2;   % Tensão fase 2
        vs3s(n) = vs3;   % Tensão fase 3
        
        % Fluxos
        fsds(n) = fsd;   % Fluxo estator fase d
        fsqs(n) = fsq;   % Fluxo estator fase q
        fsos(n) = fso;   % Fluxo de sequência 0
        frds(n) = frd;   % Fluxo rotor fase d
        frqs(n) = frq;   % Fluxo rotor fase q
        fs1s(n) = fs1;   % Fluxo fase 1
        fs2s(n) = fs2;   % Fluxo fase 2
        fs3s(n) = fs3;   % Fluxo fase 3
        
        ces(n) = ce;     % Conjugado eletromagnético 
        ves(n) = wm;     % Velocidade
        frs(n) = ws;     % Frequência
        cms(n) = cm;     % Conjugado mecânico
    end
end

figure(1),plot(Ts,is1s, Ts,is2s, Ts,is3s),zoom
title('Correntes');

figure(2),plot(Ts,vs1s, Ts,vs2s, Ts,vs3s),zoom
title('Tensões');

figure(3),plot(Ts,fs1s, Ts,fs2s, Ts,fs3s),zoom
title('Fluxos');

figure(4),plot(Ts,ces),zoom
title('Conjugado Eletromagnético');

figure(5),plot(Ts,ves),zoom
title('Velocidade');

figure(6),plot(Ts,cms),zoom
title('Conjugado de carga');

figure(7),plot(Ts, vsos),zoom
title('Tensão de sequência 0');