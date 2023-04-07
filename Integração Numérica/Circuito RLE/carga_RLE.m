clear;close all;clc
% ii(t) = k*exp(-s1*t) + ii_reg
% CARGA RLE

rr=50;        				% resistência da carga
ll=0.05;						% indutância da carga


E=10;						% tensao do barramento CC

t=0;							% tempo inicial de simulacao
h=1.0e-6;  %1.0e-7 5.0e-6     	% passo de calculo 

tf=4/60;  	% tempo total de simulaçao

tsave0=0.; 					% tempo inicial de gravaçao de vetores de dados de saida 				
tsave=tsave0;
npt=5000;  					% numero de pontos dos vetores de saida
hsave=(tf-tsave0)/npt;	% periodo de amostragem dos vetores de dados de saida 
if hsave<h
hsave=h;						% saturacao do periodo de amostragem de saida se <= passo de calcvo
npt=tf/h;
end

j=0;							% indice dos vetores de dados de saida 

% inicializacao das correntes da carga

vvm=20;
w=60*2*pi;
ii=0;				% correntes na carga

%------------------------------------------------------------------------------
%------------------------------------------------------------------------------

while t < tf					% inicio do loop de simulacao 
   
t = t + h;   						% tempo de simulaçao

vv=vvm;  %*cos(w*t); 
ii = ii + ((vv-E-rr*ii)/ll)*h;					% simulaçao da carga RLE 

if tsave <= t           % inicio gravaçao de vetores de dados de saida
tsave = tsave + hsave;
j=j+1;
T(j)=t;
svv(j)=vv;
sii(j)=ii;
end						%fim de gravaçào de vetores de dados de saida


end            		% fim do loop de simulacao

% --------------------------------------------------------------
% --------------------------------------------------------------

%Trabalhando com Laplace (i é a unidade imaginária, já que o j está sendo usado)

Vs = vvm*exp(i*0);
Ica = Vs/(rr+i*w*ll);
Icc = -E/rr;

x = linspace(0,tf,npt);
vs = abs(Vs)*cos(w.*x);
is = abs(Ica)*cos(w.*x+atan(imag(Ica)/real(Ica))) + Icc;



% traçado de algumas curvas

figure(1),subplot(2,1,1); plot(T,svv),zoom    
title('tensao');
grid

subplot(2,1,2); plot(T,sii),zoom    
title('corrente');
grid 

%pause

figure(2),
subplot(2,1,1); plot(x,vs),zoom    
title('tensao por laplace');
grid

subplot(2,1,2); plot(x,is),zoom    
title('corrente por laplace');
grid 


