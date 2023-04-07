% ___________________________________________________________
%
% Programa conversor monof�sico (fonte chaveada) - Filtro RLE + Carga RL
% ___________________________________________________________

clear;close all;clc;

%---------------- Par�metros de simula��o ------------------
t=0;						% tempo inicial de simulacao
h=1.0e-6;               	% passo de calculo 
tf=0.2;	% tempo total de simula�ao

% parametros do circuito
rr=50;        				% resist�ncia da carga
ll=0.05;					% indut�ncia da carga
cc=10.e-6;                  % capacitor do filtro
rr2=5000;                   % resist�ncia do filtro
ll2=0.5;                    % indut�ncia do filtro

E=7;						% tensao do barramento CC

%---------------- Par�metros de grava��o ------------------
tsave0=0.; 					% tempo inicial de grava�ao de vetores de dados de saida 				
tsave=tsave0;
npt=20000;  				% numero de pontos dos vetores de saida
hsave=(tf-tsave0)/npt;      % periodo de amostragem dos vetores de dados de saida 
if hsave<h
hsave=h;					% saturacao do periodo de amostragem de saida se <= passo de calcvo
npt=tf/h;
end

j=0;						% indice dos vetores de dados de saida 


%polos do circuito RLE+RL 
s12 = roots([ll*ll2*cc (rr*ll2*cc+rr2*cc*ll) (ll2+rr*rr2*cc+ll) rr+rr2]);

% inicializacao 
vv=20;
vvm=vv;
w=2*pi*60;
ii=0;				% correntes na sa�da do conversor
vc=0;               % tensao no capacitor
ii2=0;              % correntes na carga

hdisc=1e-4;         % meio periodo da dente de serra (meio periodo do PWM) 
derv=E/hdisc;       % derivada da dente de serra
vserra=E/2;         % amplitude inicial da dente de serra    
sign=-1;            % sinal inicial da derivada da dente de serra 

vcrm=3;             %amplitude inicial da tens�o de referencia na sa�da do conversor
tdisc=0;            %tempo inicial do loop PWM

vc10_int=0;         %valor inicial da integral da tens�o de polo - bra�o 1
vc20_int=0;         %valor inicial da integral da tens�o de polo - bra�o 2

%------------------------------------------------------------------------------
%------------------------------------------------------------------------------

while t < tf					% inicio do loop de simulacao 
   
    t = t + h;   						% tempo de simula�ao

    %if t >= tf/2     %transitorio de amplitude da tensao de referencia
    %    vcrm=6;
    %end
    
    % ___________________________________________________
    % loop de controle discreto - passo de calculo hdisc >> h -(realizado no computador e program�vel)
    % ___________________________________________________
   
    if t >= tdisc

        tdisc = tdisc + hdisc;  

        %inicializa��o da dente de serra
        if vserra <= -0
            vserra = -E/2;
            sign=1;
        end
        if vserra >= 0
            vserra = E/2;
            sign=-1;
        end

        %c�lculo do valor medio das tens�es de p�lo
        vc10_med = vc10_int/hdisc;
        vc20_med = vc20_int/hdisc;
        vc10_int=0;         %valor inicial da integral da tens�o de polo - bra�o 1
        vc20_int=0;         %valor inicial da integral da tens�o de polo - bra�o 2

        %tens�es de refer�ncia
        vc_ref=vcrm*cos(w*t);         %tens�o de refer�ncia na sa�da do conversor 
        vc10_ref=vc_ref/2;            %tens�o de refer�ncia de p�lo  - bra�o 1 
        vc20_ref=-vc_ref/2;           %tens�o de refer�ncia de p�lo  - bra�o 2

    end

    % ___________________________________________________
    % fim do loop de controle discreto
    % ___________________________________________________

    % ___________________________________________________
    % inicio do loop de controle cont�nuo - passo de calculo h -(realizado com circuitos dedicados, em geral n�o programaveis) 
    % ___________________________________________________

    vserra = vserra + sign*derv*h;  %dente de serra

    if vc10_ref >= vserra   %comparador para gera��o do estado das chaves - bra�o 1
        qq1=1;
    else
        qq1=0;
    end

    if vc20_ref >= vserra   %comparador para gera��o do estado das chaves - bra�o 2
        qq2=1;
    else
        qq2=0;
    end


    % ___________________________________________________
    % fim do loop de controle cont�nuo - passo de calculo h 
    % ___________________________________________________

    % ___________________________________________________
    % inicio do loop de simula��o do sistema real conversor - carga - passo de calculo h
    % ___________________________________________________

    %c�lculo das tens�es atuais (simula��o do conversor)
    vc10 = (2*qq1-1)*E/2;    %tens�o atual de p�lo - bra�o 1 
    vc20 = (2*qq2-1)*E/2;    %tens�o atual de p�lo - bra�o 2 
    vv = vc10 - vc20;        %tens�o atual na sa�da do conversor 
    
    %integra��o das tens�es de p�lo (medi��o da tens�o gerada pelo conversor)
    vc10_int = vc10_int +vc10*h;
    vc20_int = vc20_int +vc20*h;
    
    % simula��o do filtro e carga 
    
    dev_ii = (vv-vc-rr*ii)/ll;      % derivada da corrente de sa�da do conversor
    ii = ii + dev_ii*h;             % calculo da corrente de sa�da do conversor		
    
    iic = ii - ii2;                 %corrente no capacitor
    
    dev_vc = iic/cc;                % derivada da tensao na carga (capacitor)
    vc = vc + dev_vc*h;             % calculo da tensao na carga (capacitor) 
    
    dev_ii2 = (vc-rr2*ii2)/ll2;      % derivada da corrente na carga 
    ii2 = ii2 + dev_ii2*h;           % calculo da corrente na carga 
    
    
    % ___________________________________________________
    % fim do loop de simula��o do sistema conversor - carga
    % ___________________________________________________
    
    
    % _______________________________________________
    % inicio grava�ao de vetores de dados de saida
    
    if tsave <= t           
    tsave = tsave + hsave;
    j=j+1;
    T(j)=t;
    svv(j)=vv;
    sii(j)=ii;
    sii2(j)=ii2;
    svc(j)=vc;
    svc_ref(j)=vc_ref;
    svc10_med(j)=vc10_med;
    svc20_med(j)=vc20_med;
    svc10_ref(j)=vc10_ref;
    svc20_ref(j)=vc20_ref;
    svserra(j)=vserra;
    end
    
    %fim de grava��o de vetores de dados de saida
    % _______________________________________________
    

end            		% fim do loop de simulacao

% --------------------------------------------------------------
% --------------------------------------------------------------

% tra�ado de curvas

figure(1),plot(T,svv),zoom    
title('tensao de entrada');
%pause
figure(2),plot(T,sii),zoom    
title('corrente');

%pause
figure(3),plot(T,svc),zoom    
title('tensao no capacitor');


figure(5),plot(T,svc_ref,T,svc),zoom    
title('tensao de saida de referencia e atual');
%pause
%{
%pause
figure(4),plot(T,sii2),zoom    
title('corrente na carga');
%pause

figure(6),plot(T,svc10_ref,T,svc10_med),zoom    
title('tensao de p�lo de referencia e m�dia - bra�o 1');
%pause
figure(7),plot(T,svc20_ref,T,svc20_med),zoom    
title('tensao de p�lo de referencia e m�dia - bra�o 2');
%pause
%}
figure(8),plot(T,svserra,'o--'),zoom
title('portadora (dente de serra)');


