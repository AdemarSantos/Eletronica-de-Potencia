clear all; clc; close all;
format short

% CARGA RLC

rr=50;        				% resistência da carga
ll=0.05;                    % indutância da carga
cc=0.001;                   % capacitância da carga
a = rr/(2*ll);          % frequência neperiana
w0 = 1/sqrt(ll*cc);        % frequência de ressonância


t=0;							% tempo inicial de simulacao
h=1.0e-6;  %1.0e-7 5.0e-6     	% passo de calculo 

tf=8/60;  	% tempo total de simulaçao

tsave0=0.; 					% tempo inicial de gravaçao de vetores de dados de saida 				
tsave=tsave0;
npt=5000;  					% numero de pontos dos vetores de saida
hsave=(tf-tsave0)/npt;	% periodo de amostragem dos vetores de dados de saida 
if hsave<h
hsave=h;						% saturacao do periodo de amostragem de saida se <= passo de calcvo
npt=(tf-tsave0)/h;
end

j=0;							% indice dos vetores de dados de saida 

% inicializacao da corrente da carga e da tensão no capacitor

vvm=20;              % valor médio da tensão de entrada
w=60*2*pi;      % frequência angular
ii=0;				% corrente na carga
vvc = 0;           % tensão no capacitor

%------------------------------------------------------------------------------
%------------------------------------------------------------------------------

while t < tf					% inicio do loop de simulacao 
   
t = t + h;   					% tempo de simulaçao

vv=vvm;%*cos(w*t);                % tensão de entrada
vvc = vvc + ii*h/cc;            % tensão no capacitor
ii = ii + (vv-vvc-rr*ii)*h/ll;	% corrente na carga RLC 

if tsave <= t           % inicio gravaçao de vetores de dados de saida
tsave = tsave + hsave;
j=j+1;
T(j)=t;
svv(j)=vv;
svvc(j)=vvc;
sii(j)=ii;
end						%fim de gravaçào de vetores de dados de saida


end            		% fim do loop de simulacao

% --------------------------------------------------------------
% --------------------------------------------------------------

%Trabalhando com Laplace (i é a unidade imaginária, já que o j está sendo usado)
s = 0;

Vs = vvm*exp(s*0);
Is = (Vs*s*cc) / ((-ll*cc*w^2)+(rr*cc*s)+1);
Vc = (Vs) / ((ll*cc*s^2)+(rr*cc*s)+1);

x = linspace(0,tf,npt);
vs = abs(Vs)%*cos(w.*x);
is = abs(Is)%*cos(w.*x+atan(imag(Is)/real(Is)));
vc = abs(Vc)%*cos(w.*x+atan(imag(Vc)/real(Vc)));

% traçado de algumas curvas
figure(1), subplot(3,1,1); plot(T,svv),zoom    
title('tensao de entrada');
grid;

subplot(3,1,2); plot(T,svvc),zoom    
title('tensao capacitor');
grid;

subplot(3,1,3); plot(T,sii),zoom    
title('corrente na carga');
grid;

%{
figure(2),
subplot(3,1,1); plot(x,vs),zoom    
title('tensao de entrada por laplace');
grid;

subplot(3,1,2); plot(x,vc),zoom    
title('tensao capacitor por laplace');
grid;

subplot(3,1,3); plot(x,is),zoom    
title('corrente na carga por laplace');
grid;
%}
