% DUPLO CONVERSOR TRIFÁSICO - CARGA RLE

clear; close all; clc;
echo off;

%---------------- Parâmetros de Simulação ------------------
h = 1E-6;                  % Passo de cálculo
t = 0;                     % Tempo inicial de simulação
tf = 1;                    % Tempo final de simulação

%---------------- Parâmetros de Gravação ------------------
tsave0 = 0.;               % Tempo inicial de gravação
tsave = tsave0;            % Tempo de gravação
hsave = h;

n = 0;                     % Inicialização da variável de posição dos vetores de saída

%---------- Parâmetros e condições do Sistema -------------
Vgm = 90;                   % Tensão nominal da rede
f_ref = 50;                % Frequência de referência
w_ref = 2*pi*f_ref;        % Frequência angular de referência

u = 0.5;


E = 200;  %%% Tensão do barramento principal
Ea = E/2; %%% Tensão do barramento do conversor A              
Eb = E/2; %%% Tensão do barramento do conversor B

Eab = (Ea + Eb)/2;

phi = pi/10;               % Defasagem entre tensão do conversor e da rede
theta = 0;                 % Ângulo variável

%--------------------------------
Igm=20;         %corrente nominal
pg=3*Vgm*Igm;   %potencia nominal
Zgm=Vgm/Igm;    %impedacia nominal 

%parametros do indutor da fonte
rg=0.01*Zgm;
lg=0.1*Zgm/w_ref;
%--------------------------------

vg1_int = 0;               % Integral da tensão no polo 1
vg2_int = 0;               % Integral da tensão no polo 2
vg3_int = 0;               % Integral da tensão no polo 3

ig1 = 0;
ig2 = 0;
ig3 = 0;

%---------------- Inicialização das ondas triangulares ------------------
ttri = 0;             % Tempo inicial da onda triangular
ftri = 10E3;          % Frequência da onda triangular de 10 kHz
htri = 1/ftri;        % Período da onda triangular
vtri = Eab/2;         % Tensão máxima da onda triangular
dtri = Eab/htri;      % Derivada da onda triangular (dV/dT)
sign = -1;            % Sinal inicial da derivada (Comportamento decrescente)

%------------------------------------------------------------------------
% Início da simulação
while t <= tf
    t = t + h;
    
    theta = theta + h*w_ref;
    if theta >= 2*pi
        theta = theta - 2*pi;
    end
    
    eg1 = Vgm*cos(theta);
    eg2 = Vgm*cos(theta-2*pi/3);
    eg3 = Vgm*cos(theta+2*pi/3);
    
    % Início do PWM
    if ttri <= t
        ttri = ttri + htri;
        
        vg1_ref = Vgm*cos(theta-phi);
        vg2_ref = Vgm*cos(theta-(2*pi/3)-phi);
        vg3_ref = Vgm*cos(theta+(2*pi/3)-phi);
        
        vg_ref = [vg1_ref,vg2_ref,vg3_ref];
        vbamax_ref =  Eab - max(vg_ref);
        vbamin_ref = -Eab - min(vg_ref);
        
        vba_ref = u*vbamax_ref + (1-u)*vbamin_ref;
        
        vg10_ref = vg1_ref + vba_ref;
        vg20_ref = vg2_ref + vba_ref;
        vg30_ref = vg3_ref + vba_ref;
        
        % Inicialização da onda triangular
        if vtri >= 0
            vtri = Eab/2;
            sign = -1;
        else
            vtri =-Eab/2;
            sign = 1;
        end
        
        vg1_med = vg1_int/htri;
        vg2_med = vg2_int/htri;
        vg3_med = vg3_int/htri;
        
        vg1_int = 0;
        vg2_int = 0;
        vg3_int = 0;
        
    end
    % ----------------------
    % Triangular
    
    vtri = vtri + sign*dtri*h;
    vtripos = vtri + Eab/2;
    vtrineg = vtri - Eab/2;
    
    % Tensão 1
    if vg10_ref >= 0
        if vg10_ref >= vtripos
            q1a = 1;
            q1b = 0;
        else
            q1a = 1;
            q1b = 1;            
        end
    else
        if vg10_ref >= vtrineg
            q1a = 0;
            q1b = 0;            
        else
            q1a = 0;
            q1b = 1;            
        end       
    end
    
    % Tensão 2
    if vg20_ref >= 0
        if vg20_ref >= vtripos
            q2a = 1;
            q2b = 0;
        else
            q2a = 1;
            q2b = 1;            
        end
    else
        if vg20_ref >= vtrineg
            q2a = 0;
            q2b = 0;            
        else
            q2a = 0;
            q2b = 1;            
        end       
    end
    
    % Tensão 3
    if vg30_ref >= 0
        if vg30_ref >= vtripos
            q3a = 1;
            q3b = 0;
        else
            q3a = 1;
            q3b = 1;            
        end
    else
        if vg30_ref >= vtrineg
            q3a = 0;
            q3b = 0;            
        else
            q3a = 0;
            q3b = 1;            
        end       
    end
    
    % Tensões de polo na saída dos conversores
    vga10 = (2*q1a - 1)*Ea/2;
    vga20 = (2*q2a - 1)*Ea/2;
    vga30 = (2*q3a - 1)*Ea/2;
    vgb10 = (2*q1b - 1)*Eb/2;
    vgb20 = (2*q2b - 1)*Eb/2;
    vgb30 = (2*q3b - 1)*Eb/2;
    
    % Tensões entre os neutros
    vgab10 = vga10 - vgb10;
    vgab20 = vga20 - vgb20;
    vgab30 = vga30 - vgb30;
    
    % Tensões de fase geradas pelo retificador
    vba = (vgab10 + vgab20 + vgab30)/3;
    vg1 = vgab10 - vba;
    vg2 = vgab20 - vba;
    vg3 = vgab30 - vba;
    
    vg1_int = vg1_int + vg1*h;
    vg2_int = vg2_int + vg2*h;
    vg3_int = vg3_int + vg3*h;
    
    % ----------------------
    % Simulação da carga RLE
    
    dig1 = (eg1 - vg1 - rg*ig1)/lg;
	dig2 = (eg2 - vg2 - rg*ig2)/lg;
    dig3 = (eg3 - vg3 - rg*ig3)/lg;
 
	ig1 = ig1 + dig1*h;
	ig2 = ig2 + dig2*h;
    ig3 = ig3 + dig3*h;
    
    if t >= tsave
        tsave = tsave + hsave;
        n = n + 1;
        Ts(n) = t;
        
        % Tensões de polo de referência
        vg10_refs(n) = vg10_ref;
        vg20_refs(n) = vg20_ref;
        vg30_refs(n) = vg30_ref;
        
        % Tensões de fase de referência
        vg1_refs(n) = vg1_ref;
        vg2_refs(n) = vg2_ref;
        vg3_refs(n) = vg3_ref;
        
        % Tensões de fase atuais
        vg1s(n) = vg1;
        vg2s(n) = vg2;
        vg3s(n) = vg3;
        
        % Tensões de fase médias
        vg1_meds(n) = vg1_med;
        vg2_meds(n) = vg2_med;
        vg3_meds(n) = vg3_med;
        
        % Correntes de linha
        ig1s(n) = ig1;
        ig2s(n) = ig2;
        ig3s(n) = ig3;
        
        % Portadoras
        triangle(n) = vtri;
        trianglepos(n) = vtripos;
        triangleneg(n) = vtrineg;
        
        % Vba
        vba_refs(n) = vba_ref;
    end
end

%---- Tensão de saída do conversor - Braço 1
figure('Name','Tensão de saída do Conversor - B1')
plot(Ts,vg1s,Ts,vg1_meds,Ts,vg1_refs,'LineWidth',1.5),zoom
title('Tensão de saída do conversor - Braço 1','FontSize',18)
legend('vg1_{pwm}','vg1_{med}','vg1_{ref}')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid()

%---- Tensão de saída do conversor - Braço 2
figure('Name','Tensão de saída do Conversor - B2')
plot(Ts,vg2s,Ts,vg2_meds,Ts,vg2_refs,'LineWidth',1.5),zoom
title('Tensão de saída do conversor - Braço 2','FontSize',18)
legend('vg2_{pwm}','vg2_{med}','vg2_{ref}')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid()

%---- Tensão de saída do conversor - Braço 3
figure('Name','Tensão de saída do Conversor - B3')
plot(Ts,vg3s,Ts,vg3_meds,Ts,vg3_refs,'LineWidth',1.5),zoom
title('Tensão de saída do conversor - Braço 3','FontSize',18)
legend('vg3_{pwm}','vg3_{med}','vg3_{ref}')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid()

%---- Tensões de fase do conversor
figure('Name','Tensões de fase do Conversor')
plot(Ts,vg1_meds,Ts,vg2_meds,Ts,vg3_meds,'LineWidth',1.5),zoom
title('Tensões de fase do conversor','FontSize',18)
legend('vg1_{med}','vg2_{med}','vg3_{med}')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid()

%---- Tensões de polo de referência
figure('Name','Tensões de polo de referência')
plot(Ts,vg10_refs,Ts,vg20_refs,Ts,vg30_refs,'LineWidth',1.5),zoom
title('Tensões de polo do conversor A','FontSize',18)
legend('vg1_{med}','vg2_{med}','vg3_{med}')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid()

%---- Correntes de saída do conversor
figure('Name','Corrente de saída do Conversor')
plot(Ts,ig1s,Ts,ig2s,Ts,ig3s,'r-','LineWidth',1),zoom
title('Corrente de saída do conversor')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()

%---- Portadoras triangulares
figure('Name','Tensões das portadoras triangulares')
plot(Ts,trianglepos,Ts,triangleneg,'LineWidth',1.5),zoom
title('Tensões de saída do conversor','FontSize',18)
legend('Triangular Positiva','Triangular Negativa')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
axis([0.997 1 -110 110])
grid()