
% ___________________________________________________________
%
% Programa conversor trifásico - Fonte RLE (sistema de tensão trifásico)
% ___________________________________________________________

clear; 
close all;
clc;


t=0;            %tempo inicial de simulacao
tmax=0.2;       %tempo final simulacao
h=1.e-7;        %passo de calculo 

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
Vgm=90;     %tensao nominal da fonte
Ed=200;  %tensao no barramento 
Igm=20;         %corrente nominal
pg=3*Vgm*Igm;   %potencia nominal
Zgm=Vgm/Igm;    %impedacia nominal 

%parametros do indutor da fonte
rg=0.01*Zgm;
lg=0.1*Zgm/wg;

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


Ea = Ed/2;  %100
Eb = Ed/2;  %100
Eab= (Ea + Eb)/2;  %100
der=Ed/hpwm;  %derivada da dente de serra
der=Eab/hpwm;
vtrig=Ed/2;
vtrig=Eab/2;
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
%             vgref = [vg1ref, vg2ref, vg3ref];
%             vbaref_max =  Eab/2 - max(vgref);
%             vbaref_min = -Eab/2 - min(vgref);
%             vbaref_med = (vbaref_max+vbaref_min)/2;

            %geração das tensões de pólo de referencia
            vbaref_med2=0; 
            vg10ref=vg1ref+vbaref_med2;  
            vg20ref=vg2ref+vbaref_med2;
            vg30ref=vg3ref+vbaref_med2;

            %inicializacao da onda triangular
            %der=(Eab)/hpwm;
             %Eab= 100
            if vtrig>=0
                vtrig=Eab/2;  
                sig=-1;
            end
            if vtrig<=0
                vtrig=-Eab/2; 
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
    
     %triangular
    vtrig=vtrig+sig*der*h;     

    vtrigpos = vtrig + Eab/2;
    vtrigneg = vtrig - Eab/2; 

    %calculo da tensoes de polo

    %% Tensao 1 
    if (vg10ref > 0)
     
        if (vg10ref >= vtrigpos)
            q1a = 1; 
            q1b = 0;
        else
            q1a = 1; 
            q1b = 1;
        end
     
    else
    
        if (vg10ref >= vtrigneg)
            q1a = 0; 
            q1b = 0;
        else
            q1a = 0; 
            q1b = 1;
        end
    
    end
    %% Tensao 2  
    q1ab=q1a-q1b;
    
    if (vg20ref > 0)
     
        if (vg20ref >= vtrigpos)
            q2a=1; 
            q2b=0;
        else
            q2a=1; 
            q2b =1;
        end
     
    else
    
        if (vg20ref >= vtrigneg)
            q2a=0; 
            q2b=0;
        else
            q2a=0; 
            q2b =1;
        end
    
    end

    %% Tensao 3
    if (vg30ref > 0)
     
        if (vg30ref >= vtrigpos)
            q3a=1; 
            q3b=0;
        else
            q3a=1; 
            q3b =1;
        end
     
    else
    
        if (vg30ref >= vtrigneg)
            q3a=0; 
            q3b=0;
        else
            q3a=0; 
            q3b =1;
        end
    end

    %calculo tensoes de polo atuais do conversor    
    vga10 = (2*q1a-1)*Ea/2;  
    vga20 = (2*q2a-1)*Ea/2;  
    vga30 = (2*q3a-1)*Ea/2;
    vgb10 = (2*q1b-1)*Eb/2;  
    vgb20 = (2*q2b-1)*Eb/2;  
    vgb30 = (2*q3b-1)*Eb/2; 
    
    %tensao entre neutros
    
    vgab10 = vga10 - vgb10;
    vgab20 = vga20 - vgb20;
    vgab30 = vga30 - vgb30;
 
        
    %tensoes de fase geradas pelo retificador
    vba = 0;
    vg1 = vgab10-vba;  
    vg2 = vgab20-vba;
    vg3 = vgab30-vba;


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
        triangpos(j)=vtrigpos;
        triangneg(j)=vtrigneg;
        svgab10(j)=vgab10;
        svgab20(j)=vgab20;
        svgab30(j)=vgab30;
        svba(j)=vba;
        svg10ref(j)=vg10ref;
        sq1a(j)=q1a;
        sq1b(j)=q1b;
        sq1ab(j)=q1ab;
    end
   
	
end   
%___________________________________
%fim do loop de simulacao


%___________________________________
% traçado de curvas

figure(1),plot(tempo,corrente1,tempo,corrente2,tempo,corrente3),grid,zoom
title('correntes');
%pause

figure(2),plot(tempo,tensaog1ref,tempo,tensaog2ref,tempo,tensaog3ref),grid,zoom
title('tensoes ref');
%pause

figure(3),plot(tempo,tensao1,tempo,tensao2,tempo,tensao3),grid,zoom
title('tensoes de fase gerada pelo conversor');
%pause

figure(4),plot(tempo,tensao1),grid,zoom
title('tensão de fase gerada pelo converso - fase 1');
%pause

figure(5),plot(tempo,tensaog1ref,tempo,tensaog1_m),grid,zoom
title('tensoes ref e medida - fase 1');
%pause

figure(6),plot(tempo,tensaog2ref,tempo,tensaog2_m),grid,zoom
title('tensoes ref e medida - fase 2');
%pause

figure(7),plot(tempo,tensaog3ref,tempo,tensaog3_m),grid,zoom
title('tensoes ref e medida - fase 3');

figure(8),plot(tempo,tensao1,tempo,svg10ref),grid,zoom

figure(9),plot(tempo,triangpos,tempo,triangneg,tempo,svg10ref),grid,zoom
title('Portadoras e tensão de ref na fase 1');

