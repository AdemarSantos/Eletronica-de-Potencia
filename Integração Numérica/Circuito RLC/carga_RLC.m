clear all; clc; close all;
format short

% CARGA RLC

rr=50;        				% resist�ncia da carga
ll=0.05;                    % indut�ncia da carga
cc=0.001;                   % capacit�ncia da carga
a = rr/(2*ll);          % frequ�ncia neperiana
w0 = 1/sqrt(ll*cc);        % frequ�ncia de resson�ncia


t=0;							% tempo inicial de simulacao
h=1.0e-6;  %1.0e-7 5.0e-6     	% passo de calculo 

tf=8/60;  	% tempo total de simula�ao

tsave0=0.; 					% tempo inicial de grava�ao de vetores de dados de saida 				
tsave=tsave0;
npt=5000;  					% numero de pontos dos vetores de saida
hsave=(tf-tsave0)/npt;	% periodo de amostragem dos vetores de dados de saida 
if hsave<h
hsave=h;						% saturacao do periodo de amostragem de saida se <= passo de calcvo
npt=(tf-tsave0)/h;
end

j=0;							% indice dos vetores de dados de saida 

% inicializacao da corrente da carga e da tens�o no capacitor

vvm=20;              % valor m�dio da tens�o de entrada
w=60*2*pi;      % frequ�ncia angular
ii=0;				% corrente na carga
vvc = 0;           % tens�o no capacitor

%------------------------------------------------------------------------------
%------------------------------------------------------------------------------

while t < tf					% inicio do loop de simulacao 
   
t = t + h;   					% tempo de simula�ao

vv=vvm;%*cos(w*t);                % tens�o de entrada
vvc = vvc + ii*h/cc;            % tens�o no capacitor
ii = ii + (vv-vvc-rr*ii)*h/ll;	% corrente na carga RLC 

if tsave <= t           % inicio grava�ao de vetores de dados de saida
tsave = tsave + hsave;
j=j+1;
T(j)=t;
svv(j)=vv;
svvc(j)=vvc;
sii(j)=ii;
end						%fim de grava��o de vetores de dados de saida


end            		% fim do loop de simulacao

% --------------------------------------------------------------
% --------------------------------------------------------------

%Trabalhando com Laplace (i � a unidade imagin�ria, j� que o j est� sendo usado)
s = 0;

Vs = vvm*exp(s*0);
Is = (Vs*s*cc) / ((-ll*cc*w^2)+(rr*cc*s)+1);
Vc = (Vs) / ((ll*cc*s^2)+(rr*cc*s)+1);

x = linspace(0,tf,npt);
vs = abs(Vs)%*cos(w.*x);
is = abs(Is)%*cos(w.*x+atan(imag(Is)/real(Is)));
vc = abs(Vc)%*cos(w.*x+atan(imag(Vc)/real(Vc)));

% tra�ado de algumas curvas
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
