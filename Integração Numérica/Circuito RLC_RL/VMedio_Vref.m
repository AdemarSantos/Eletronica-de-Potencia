clear;close all;clc;
format long

h = 1E-7;                % Passo de cálculo
t0 = 0.0022;             % Tempo inicial de simulação     
t  = t0;                 % Tempo de simulação
tf = 0.0023;             % Tempo final de simulação
%---------------- Inicialização da onda triangular ------------------
E = 10;
tT = t0;                 % Tempo inicial da onda triangular
fT = 10E3;               % Frequência da onda triangular de 10 kHz
hT = 1/fT;               % Período da onda triangular
VT = -E/2;               % Tensão máxima da onda triangular
dT = E/hT;               % Derivada da onda triangular (dV/dT)
sign = 1;                % Sinal inicial da derivada (Comportamento decrescente)

%---------------- Parâmetros de Gravação ------------------
tsave0 = 0.;             % Tempo inicial de gravação
tsave = tsave0;          % Tempo de gravação
npt = 40000;             % Dimensão do vetor de saída de dados
hsave = (tf-tsave0)/npt; % Passo de de gravação dos vetores de saída de dados

if hsave < h             % Sendo o Passo de gravação menor que o passo de cálculo (Saturação)
    hsave = h;           % Defina o passo de gravação = passo de cálculo
    npt = (tf-tsave0)/hsave; % Recalcule o a dimensão dos vetores de saída de dados
end

n = 0;
cont1=0;
cont2=0;

while(t < tf)
    
    if t >= tT
        tT = tT + hT;
        
        if VT <= 0
            VT = -E/2;
            sign = 1;
        else
            VT = E/2;
            sign = -1;
        end
    end
    VT = VT + sign*dT*h;
    V_ref = (E/2)*cos(60*2*pi*t);
    
    if V_ref > VT
        cont1 = cont1 + 1; % Quantidade de pontos de tempo com a chave ligada
    else
        cont2 = cont2 + 1; % Quantidade de pontos de tempo com a chave desligada
    end
    
    if tsave <= t    
        n = n + 1;
        Ts(n) = t;
        VTs(n)= VT;
        V_refs(n) = V_ref;
    end
    t = t + h;
end
contt = cont1 + cont2;           % Quantidade total de pontos no período

dc = cont1/contt;                % Duty cycle
tint = (tf-t0) * cont1/contt;    % Tempo, dentro do período (tf-t0) que a chave permanece ligada

Vmed = (E)*dc - (E/2)            % Tensão média
Vref = (E/2)*cos(60*2*pi*(t0+(tf-t0)/2)) % Tensão de referência calculada no tempo intermediário do período

plot(Ts,VTs,Ts,V_refs,'r-',[t0 tf],[Vmed Vmed],'k-')
legend(' Tensão da portadora triangular','Tensão de referência','Tensão Média','Location','SouthEast')
xlabel("Tempo (s)")
ylabel("Tensão (V)")
grid()
