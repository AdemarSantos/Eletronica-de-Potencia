
% ___________________________________________________________
%
% Programa conversor trifásico - Fonte RLE (sistema de tensão trifásico)
% ___________________________________________________________

clear all

t=0;            %tempo inicial de simulacao
tmax=0.3;       %tempo final simulacao
h=1.e-6;        %passo de calculo 

% parametros da amarzenagem de curvas
hp=tmax/20000;  	
if(hp < h),     %passo de amostragem das variaveis de saida
hp=h;
end
tp=0;
j=0;

% constantes
pi23=2*pi/3;
rq23=sqrt(2/3);
rq3=sqrt(3);
rq2=sqrt(2);

% parâmetros da fonte RLE
wg=2*pi*50;          %frequencia da tensao da fonte
Vgm=220*sqrt(2);     %tensao nominal da fonte
Ed=1.1*sqrt(3)*Vgm;  %tensao no barramento 
Igm=20;         %corrente nominal
pg=3*Vgm*Igm;   %potencia nominal
Zgm=Vgm/Igm;    %impedacia nominal 

%parametros do indutor da fonte
rg=0.01*Zgm
lg=0.1*Zgm/wg

% parametros inicias do sistema
tete=0;
ig1=0;
q1=0;
ig2=0;
q2=0;
ig3=0;
q3=0;

% parâmetros do PWM
hpwm=1.e-4;  %periodo  de chaveamento do PWM 
tpwm=0;

der=Ed/hpwm;  %derivada da dente de serra
vtrig=Ed/2;
sign=-1;

%variavel para medição das tensões geradas pelo conversor
vg1_i = 0;  
vg2_i = 0;
vg3_i = 0; 

%
%MALHA DE SIMULACAO DO MOTOR
%

%___________________________________
%início do loop de simulacao

while t<=tmax,
	%
	t=t+h;		%tempo real de simulacao
	%---------------------------------------
	%---------------------------------------
	%fonte de tensao de alimentacao
	%---------------------------------------
	%---------------------------------------
	%
	tete=tete+h*wg;						%angulo da fonte RLE
	if tete >= 2*pi,
	tete=tete-2*pi;
    end
      
    eg1=Vgm*cos(tete);   				%tensões da fonte RLE
	eg2=Vgm*cos(tete-pi23);
	eg3=Vgm*cos(tete+pi23);
    
    %______________________________________________________________________
    % Inicio do loop de controle discreto (PWM)
   
    if t>=tpwm
        
            tpwm=tpwm+hpwm;
            
            %tensões de referencia do conversor trifásico
            vg1ref=Vgm*cos(tete-pi/10);   				
            vg2ref=Vgm*cos(tete-pi23-pi/10);
            vg3ref=Vgm*cos(tete+pi23-pi/10);
            
            %calculos da referencia de tensão entre neutros (fonte - conversor)  
            vgref = [vg1ref, vg2ref, vg3ref];
            vn0ref_max=Ed/2 - max(vgref);
            vn0ref_min=-Ed/2 - min(vgref);
            vn0ref_med=(vn0ref_max+vn0ref_min)/2;

            %geração das tensões de pólo de referencia
            vg10ref=vg1ref+vn0ref_med;  
            vg20ref=vg2ref+vn0ref_med;
            vg30ref=vg3ref+vn0ref_med;

            %inicializacao da onda triangular
            if vtrig>=0
                vtrig=Ed/2;
                sig=-1;
            end
            if vtrig<=0
                vtrig=-Ed/2;
                sig=1;
            end

            %medição de tensao (comparação com as referencias vg10ref a vg30ref)
            vg1_m=vg1_i/hpwm;     
            vg2_m=vg2_i/hpwm;
            vg3_m=vg3_i/hpwm;
            vg1_i=0;
            vg2_i=0;
            vg3_i=0;
        
    end    
   
    % Fim do loop de controle discreto (PWM)
    %______________________________________________________________________
   
    %______________________________________________________________________
    % simulação do conversor
    
    vtrig=vtrig+sig*der*h;  %dente de serra
    
    %cálculo da tensoes de pólo
    if vg10ref >= vtrig
        q1=1;
    else
        q1=0;
    end
    if vg20ref >= vtrig
        q2=1;
    else
        q2=0;
    end
    if vg30ref >= vtrig
        q3=1;
    else
        q3=0;
    end
  
	%---------------------------------------
    %cálculo tensoes de pólo atuais do conversor    
    vg10=(2*q1-1)*Ed/2;  
    vg20=(2*q2-1)*Ed/2;
    vg30=(2*q3-1)*Ed/2;
    
    %tensão entre neutros
    vn0=(vg10+vg20+vg30)/3;  
    
    vg1=vg10-vn0;        %tensoes de fase geradas pelo retificador        
    vg2=vg20-vn0;
    vg3=vg30-vn0;
    
    vg1_i = vg1_i + vg1*h;  %medição de tensao
    vg2_i = vg2_i + vg2*h;
    vg3_i = vg3_i + vg3*h;

	
	%______________________________________________________________________
  	%simulação do circuito conversor mais fonte RLE
    
	dev_ig1 = (eg1 - vg1 - rg*ig1)/lg;
	dev_ig2 = (eg2 - vg2 - rg*ig2)/lg;
    dev_ig3 = (eg3 - vg3 - rg*ig3)/lg;
 
	ig1 = ig1 + dev_ig1*h;
	ig2 = ig2 + dev_ig2*h;
    ig3 = ig3 + dev_ig3*h;
	
 	%armazenagem das variaveis de saida 
	if t > tp,
	j=j+1;
	tp=tp+hp;
	
	tempo(j) = t;
   	corrente1(j) = ig1;
	corrente2(j) = ig2;
	corrente3(j) = ig3;
	tensao1(j) = vg1;
	tensao2(j) = vg2;
	tensao3(j) = vg3;
    tensaoe1(j)=eg1;
    tensaoe2(j)=eg2;
    tensaoe3(j)=eg3;
    tensaog1_m(j)=vg1_m;
    tensaog2_m(j)=vg2_m;
    tensaog3_m(j)=vg3_m;
    tensaog1ref(j)=vg1ref;
    tensaog2ref(j)=vg2ref;
    tensaog3ref(j)=vg3ref;
    triang(j)=vtrig;
    end
   
	
end   
%___________________________________
%fim do loop de simulacao


%___________________________________
% traçado de curvas

figure(1),plot(tempo,corrente1,tempo,corrente2,tempo,corrente3),grid,zoom
title('correntes');
% pause

figure(2),plot(tempo,tensaog1ref,tempo,tensaog2ref,tempo,tensaog3ref),grid,zoom
title('tensoes ref');
% pause

figure(3),plot(tempo,tensao1,tempo,tensao2,tempo,tensao3),grid,zoom
title('tensoes de fase gerada pelo conversor');
% pause

figure(4),plot(tempo,tensao1),grid,zoom
title('tensão de fase gerada pelo converso - fase 1');
% pause

figure(5),plot(tempo,tensaog1ref,tempo,tensaog1_m),grid,zoom
title('tensoes ref e medida - fase 1');
% pause

figure(6),plot(tempo,tensaog2ref,tempo,tensaog2_m),grid,zoom
title('tensoes ref e medida - fase 2');
% pause

figure(7),plot(tempo,tensaog3ref,tempo,tensaog3_m),grid,zoom
title('tensoes ref e medida - fase 3');



