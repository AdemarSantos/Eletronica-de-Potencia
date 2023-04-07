%
echo off
% SIMULACAO DO MOTOR DE INDUCAO
% PROGRAMA: SIMI.M (24/05/04)
%
% CONDICOES INICIAIS
%
%constantes
%
pi23=2*pi/3;
rq23=sqrt(2/3);
rq3=sqrt(3); 
%parametros da maquina 0
rs=0.39;
rr=1.41;  %1.41
ls=0.094;
lr=0.094;
msr=0.091;
%lso=0.1*ls
%
jm = 0.04;
kf = 0.01;
cte_tempo_mec=jm/kf;
idt=1/(ls*lr-msr*msr);
p = 2;   %numero de pares de polos
amsr = p*idt*msr;
%
%parametros da simulacao		
h=1.e-4;  % 1.e-4; 2.e-4; 5.e-4;
%h=input('entre com o periodo de discretizacao h  ')
%%
tmax=1;
%tmax=input('entre com o tempo de simulacao (tmax)  ')
kwm=40/316; 				%kwm=40/316
hp=tmax/2000;  	%hp=tmax/2000;
if(hp < h),     	%passo de amostragem das variaveis de saida
hp=h;
end
%
%condicoes iniciais
cm=0;		%conjugado mecanico
wm=0.;		%velocidade da maquina
t=0;
tp=0;
j=0;
ce = 0; 
ws=377;
Vsm=220*sqrt(2);  
Vs=Vsm;
tete=0;
fsd=0;
fsq=0;
frd=0;
frq=0;
isd=0;
isq=0;
ird=0;
irq=0;
%
% modelo estado mecanico

%
%MALHA DE SIMULACAO DO MOTOR
%
while t<=tmax,
	%
	t=t+h;		%tempo real de simulacao
	%---------------------------------------
	%---------------------------------------
	%fonte de tensao de alimentacao
	%---------------------------------------
	%---------------------------------------
	%
	tete=tete+h*ws;						%angulo da tensao
	if tete >= 2*pi,
	tete=tete-2*pi;
	end
	vs1=Vs*cos(tete);   				%tensões
	vs2=Vs*cos(tete-pi23);
	vs3=Vs*cos(tete+pi23);
	%inversao de velocidade
	%---------------------------------------
	%---------------------------------------
	%maquina de indução
	%---------------------------------------
	%---------------------------------------
	vsd = rq23.*(vs1 - vs2./2 - vs3./2); 
	vsq = rq23.*(vs2*rq3./2 - vs3*rq3./2);
	%vso=
    
	dervfsd = vsd - rs*isd;
	dervfsq = vsq - rs*isq;
	dervfrd = -rr*ird - frq*wm;
	dervfrq = -rr*irq + frd*wm;
	%deriso=
	fsd = fsd + dervfsd*h;
	fsq = fsq + dervfsq*h;
	frd = frd + dervfrd*h;
	frq = frq + dervfrq*h;
	%iso=iso+deriso*h;
    %fso=lso*iso;
    
	ce = amsr*(fsq*frd-fsd*frq);
	
	isd=idt*(lr*fsd - msr*frd);
	isq=idt*(lr*fsq - msr*frq);
	
	ird=idt*(-msr*fsd + ls*frd);
	irq=idt*(-msr*fsq + ls*frq);
	
	is1=rq23*isd;  %k*iso
	is2=rq23*(-isd./2 + rq3*isq/2); %k*iso
	is3=rq23*(-isd./2 - rq3*isq/2);  %k*iso
	
	fs1=rq23*fsd;  %k*fso
	fs2=rq23*(-fsd./2 + rq3*fsq./2); %k*fso
	fs3=rq23*(-fsd./2 - rq3*fsq./2);  %k*fso
	
	%equacao de estado mecanica discreta
	derwm = - wm/cte_tempo_mec + p*(ce-cm)/jm;
	wm = wm + derwm*h;       
	%
	if t >= tmax/2,
	cm=40;
	end
	%armazenagem das variaveis de saida 
	if t > tp,
	j=j+1;
	tp=tp+hp;
	
	tempo(j) = t;
	corrented(j) = isd;
	correnteq(j) = isq;
	corrente1(j) = is1;
	corrente2(j) = is2;
	corrente3(j) = is3;
	tensao1(j) = vs1;
	tensao2(j) = vs2;
	tensao3(j) = vs3;
	tensaosd(j) = vsd;
	tensaosq(j) = vsq;
	fluxord(j) = frd;
	fluxorq(j) = frq;
	fluxos1(j) = fs1;
	fluxos2(j) = fs2;
	fluxos3(j) = fs3;
	fluxosd(j) = fsd;
	fluxosq(j) = fsq;
	conjugado(j) = ce;
	velocidade(j) = wm;
	frequencia(j) = ws;
	conjcarga(j) = cm;
	end
	%
end   %fim do while
%
figure(1),plot(tempo,corrente1,tempo,corrente2,tempo,corrente3),zoom
title('correntes');
% pause

figure(2),plot(tempo,tensao1,tempo,tensao2,tempo,tensao3),zoom
title('tensoes');
% pause

figure(3),plot(tempo,fluxos1,tempo,fluxos2,tempo,fluxos3),zoom
title('fluxos');
% pause


figure(4),plot(tempo,conjugado),zoom
title('conjugado eletromagnético');
% pause

figure(5),plot(tempo,velocidade),zoom
title('velocidade');
% pause

figure(6),plot(tempo,conjcarga),zoom
title('conjugado de carga');
% pause

