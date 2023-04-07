% SIMULAÇÃO DO CONVERSOR TRIFÁSICO

clear;close all;clc;
echo off;

%---------------- Parâmetros de Simulação ------------------
h = 1E-6;                  % Passo de cálculo
t = 0;                     % Tempo inicial de simulação
tf = 1;

%---------------- Parâmetros de Gravação ------------------
tsave0 = 0.;               % Tempo inicial de gravação
tsave = tsave0;            % Tempo de gravação
hsave = h;

% npt = 5000;              % Dimensão do vetor de saída de dados
% 
% hsave = (tf-tsave0)/npt; % Passo de de gravação dos vetores de saída de dados
% 
% if hsave < h             % Sendo o Passo de gravação menor que o passo de cálculo (Saturação)
%     hsave = h;           % Defina o passo de gravação = passo de cálculo
%     npt = (tf-tsave0)/hsave; % Recalcule o a dimensão dos vetores de saída de dados
% end

n = 0;                     % Inicialização da variável de posição dos vetores de saída

%---------- Parâmetros e condições do Sistema -------------
V_ref = 220;               % Tensão de referência trifásica

E = V_ref*sqrt(3)*1.05;    % Tensão do barramento CC
u = 0.5 ;                  % Definição do fator de repartição

V1_ref = V_ref;
V2_ref = V_ref;
V3_ref = V_ref;            % Equilibrio de tensões

f_ref = 60;                % Frequência de referência
w_ref = 2*pi*f_ref;        % Frequência angular de referência

%%%%%%%%%%%%%%%%%%%
Vgm = 220*sqrt(2);
Igm=20;
Zgm=Vgm/Igm;
rg=0.01*Zgm;
lg=0.1*Zgm/w_ref;
%%%%%%%%%%%%%%%%%%

i1 = 0;                    % Corrente inicial no ramo 1
i2 = 0;                    % Corrente inicial no ramo 2
i3 = 0;                    % Corrente inicial no ramo 3

v10_int = 0;               % Integral da tensão no polo 1
v20_int = 0;               % Integral da tensão no polo 2
v30_int = 0;               % Integral da tensão no polo 3

tete = 0;

v10_ref = 0;
v20_ref = 0;
v30_ref = 0;
vn0_int = 0;

%---------------- Inicialização da onda triangular ------------------
ttriangle = 0;             % Tempo inicial da onda triangular
ftriangle = 10E3;          % Frequência da onda triangular de 10 kHz
htriangle = 1/ftriangle;   % Período da onda triangular
vtriangle = E/2;           % Tensão máxima da onda triangular
dtriangle = E/htriangle;   % Derivada da onda triangular (?V/?T)
sign = -1;                 % Sinal inicial da derivada (Comportamento decrescente)

%--------------------------------------------------------------------
%--------------------------------------------------------------------
while t < tf
    t = t + h;
    
    % Progressão dos ângulos das tensões
	tete = tete + h*w_ref;
	if tete >= 2*pi
        tete = tete - 2*pi;
    end
    
    % Tensões da fonte RLE  
    eg1 = Vgm*cos(tete);   				
	eg2 = Vgm*cos(tete - 2*pi/3);
	eg3 = Vgm*cos(tete - 4*pi/3);
    
    % Tensões da rede
    vs1 = V1_ref*cos(tete);   				
	vs2 = V2_ref*cos(tete-2*pi/3);
	vs3 = V3_ref*cos(tete+2*pi/3);
    
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
        
        % Tensões médias
        v10_med = v10_int/htriangle;
        v20_med = v20_int/htriangle;
        v30_med = v30_int/htriangle;
        vn0_med = vn0_int/htriangle;
        
        % Reset do integrador
        v10_int = 0;
        v20_int = 0;
        v30_int = 0;
        vn0_int = 0;
        
        % Sequência positiva
        v1_ref = V1_ref*cos(w_ref*t);
        v2_ref = V2_ref*cos(w_ref*t - 2*pi/3);
        v3_ref = V3_ref*cos(w_ref*t - 4*pi/3);

        % Fator de repartição
        Vn0 = [v1_ref,v2_ref,v3_ref];
        vn0max =  E/2 - max(Vn0);
        vn0min = -E/2 - min(Vn0);
        vn0_ref = u*vn0max + (1-u)*vn0min;
        
        % Tensões de polo de referência
        
        v10_ref = v1_ref + vn0_ref;
        v20_ref = v2_ref + vn0_ref;
        v30_ref = v3_ref + vn0_ref;

    end
    % Progressão da onda triangular
    vtriangle = vtriangle + sign*dtriangle*h;
    
    % Estado das chaves
    if v10_ref >= vtriangle % Estado das chaves (Braço 1)
        q1 = 1; % Chave fechada (IGBT em condução)
    else
        q1 = 0; % Chave aberta (IGBT em corte)
    end
    
    if v20_ref >= vtriangle % Estado das chaves (Braço 2)
        q2 = 1; % Chave fechada (IGBT em condução)
    else
        q2 = 0; % Chave aberta (IGBT em corte)
    end
    
    if v30_ref >= vtriangle % Estado das chaves (Braço 3)
        q3 = 1; % Chave fechada (IGBT em condução)
    else
        q3 = 0; % Chave aberta (IGBT em corte)
    end
    
    % Tensões de polo
    v10 = (2*q1 - 1)*(E/2);
    v20 = (2*q2 - 1)*(E/2);
    v30 = (2*q3 - 1)*(E/2);
    
    % Integração das tensões de polo
    v10_int = v10_int + v10*h;
    v20_int = v20_int + v20*h;
    v30_int = v30_int + v30*h;
    
    if t >= tf/2
        % Estado da chave
        if vn0_ref >= vtriangle % Estado das chaves (Braço n0)
            qn0 = 1; % Chave fechada (IGBT em condução)
        else
            qn0 = 0; % Chave aberta (IGBT em corte)
        end
        % Tensão de polo
        vn0 = (2*qn0 - 1)*(E/2);
    else
        vn0 = (1/3)*(v10+v20+v30);
    end
    
    vn0_int = vn0_int + vn0*h;
    % Tensões nos braços
    
    v1 = v10 - vn0;
    v2 = v20 - vn0;
    v3 = v30 - vn0;
    
    dev_i1 = (eg1 - v1 - rg*i1)/lg;
	dev_i2 = (eg2 - v2 - rg*i2)/lg;
    dev_i3 = (eg3 - v3 - rg*i3)/lg;
 
	i1 = i1 + dev_i1*h;
	i2 = i2 + dev_i2*h;
    i3 = i3 + dev_i3*h;

    % Salvamento das variáveis
    if tsave <= t
        tsave = tsave + hsave;
        n = n + 1;
        % Tempo
        Ts(n) = t;        

        % Tensões da rede
        vs1s(n) = vs1;
        vs2s(n) = vs2;
        vs3s(n) = vs3;
        
        % Tensões de referência
        v1_refs(n) = v1_ref;
        v2_refs(n) = v2_ref;
        v3_refs(n) = v3_ref;
        vn0_refs(n) = vn0_ref;
        
        % Tensões de fase na saída do conversor
        v1s(n) = v1;
        v2s(n) = v2;
        v3s(n) = v3;
        vn0s(n)= vn0;
        
        % Correntes de linha na saída do conversor
        i1s(n) = i1;
        i2s(n) = i2;
        i3s(n) = i3;
        
        % Tensões médias de polo
        v10_meds(n) = v10_med;
        v20_meds(n) = v20_med;
        v30_meds(n) = v30_med;
        vn0_meds(n) = vn0_med;
        
        % Sinal triangular
        vtriangles(n) = vtriangle;
    end
end

% % SIMULAÇÃO DO LADO 1
% 
% %---- Tensão de saída do conversor - Braço 1
% figure('Name','Tensão de saída do Conversor - B1')
% plot(Ts,v1s,Ts,v10_meds-vn0_meds,'r-','LineWidth',1.5),zoom
% title('Tensão de saída do conversor - Braço 1','FontSize',18)
% legend('vg_{pwm}','vg_{med}')
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% %axis([0.1 0.2 -410 410])
% grid()
% 
% %---- Corrente do circuito lado 1
% figure('Name','Corrente de saída do Conversor - B1')
% plot(Ts,i1s,'r-','LineWidth',1),zoom
% title('Corrente de saída do conversor - Braço 1')
% xlabel("Tempo (s)")
% ylabel("Corrente (A)")
% grid()
% 
% % SIMULAÇÃO DO LADO 2
% 
% %---- Tensão de saída do conversor - Braço 2
% figure('Name','Tensão de saída do Conversor - B2')
% plot(Ts,v2s,Ts,v20_meds-vn0_meds,'r-','LineWidth',1.5),zoom
% title('Tensão de saída do Conversor - Braço 2','FontSize',18)
% legend('vg_{pwm}','vg_{med}')
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% %axis([0.1 0.20 -410 410])
% grid()
% 
% %---- Corrente do circuito lado 2
% figure('Name','Corrente de saída do conversor - B2')
% plot(Ts,i2s,'r-','LineWidth',1),zoom
% title('Corrente de saída do conversor - Braço 2')
% xlabel("Tempo (s)")
% ylabel("Corrente (A)")
% grid()
% 
% % SIMULAÇÃO DO LADO 3
% 
% %---- Tensão de saída do conversor - Braço 3
% figure('Name','Tensão de saída do conversor - B3')
% plot(Ts,v3s,Ts,v30_meds-vn0_meds,'r-','LineWidth',1.5),zoom
% title('Tensão de saída do conversor - Braço 3','FontSize',18)
% legend('vg_{pwm}','vg_{med}')
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% %axis([0.1 0.20 -410 410])
% grid()
% 
% %---- Corrente do circuito lado 3
% figure('Name','Corrente de saída do conversor - B3')
% plot(Ts,i3s,'r-','LineWidth',1),zoom
% title('Corrente de saída do conversor - Braço 3')
% xlabel("Tempo (s)")
% ylabel("Corrente (A)")
% grid()
% 
% % SIMULAÇÃO DO LADO N
% 
% %---- Tensão de saída do conversor - Braço N
% figure('Name','Tensão de saída do conversor - BN')
% plot(Ts,vn0s,Ts,vn0_meds,'r-','LineWidth',1.5),zoom
% title('Tensão de saída do conversor - Braço N','FontSize',18)
% legend('vg_{pwm}','vg_{med}')
% xlabel("Tempo (s)")
% ylabel("Tensão (V)")
% %axis([0.1 0.20 -410 410])
% grid()
% 
% %---- Corrente do circuito lado N
% figure('Name','Corrente de saída do conversor - BN')
% plot(Ts,i1s+i2s+i3s,'r-','LineWidth',1),zoom
% title('Corrente de saída do conversor - Braço N')
% xlabel("Tempo (s)")
% ylabel("Corrente (A)")
% grid()
% 
% %---- Correntes
% figure('Name','Correntes de saída do conversor')
% plot(Ts,i1s,Ts,i2s,Ts,i3s,'r-','LineWidth',1),zoom
% title('Correntes de saída do conversor')
% xlabel("Tempo (s)")
% ylabel("Corrente (A)")
% grid()


figure(1),plot(Ts,i1s,Ts,i2s,Ts,i3s),grid,zoom
title('correntes');
% pause

figure(2),plot(Ts,v1_refs,Ts,v2_refs,Ts,v3_refs),grid,zoom
title('tensoes ref');
% pause

figure(3),plot(Ts,v1s,Ts,v2s,Ts,v3s),grid,zoom
title('tensoes de fase gerada pelo conversor');
% pause

figure(4),plot(Ts,v1s),grid,zoom
title('tensão de fase gerada pelo converso - fase 1');
% pause

figure(5),plot(Ts,v1_refs,'b-',Ts,v10_meds-vn0_meds,'r-'),grid,zoom
title('tensoes ref e medida - fase 1');
% pause

figure(6),plot(Ts,v2_refs,'b-',Ts,v20_meds-vn0_meds,'r-'),grid,zoom
title('tensoes ref e medida - fase 2');
% pause

figure(7),plot(Ts,v3_refs,'b-',Ts,v30_meds-vn0_meds,'r-'),grid,zoom
title('tensoes ref e medida - fase 3');