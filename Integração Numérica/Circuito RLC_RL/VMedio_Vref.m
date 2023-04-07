clear;close all;clc;
format long

h = 1E-7;                % Passo de c�lculo
t0 = 0.0022;             % Tempo inicial de simula��o     
t  = t0;                 % Tempo de simula��o
tf = 0.0023;             % Tempo final de simula��o
%---------------- Inicializa��o da onda triangular ------------------
E = 10;
tT = t0;                 % Tempo inicial da onda triangular
fT = 10E3;               % Frequ�ncia da onda triangular de 10 kHz
hT = 1/fT;               % Per�odo da onda triangular
VT = -E/2;               % Tens�o m�xima da onda triangular
dT = E/hT;               % Derivada da onda triangular (dV/dT)
sign = 1;                % Sinal inicial da derivada (Comportamento decrescente)

%---------------- Par�metros de Grava��o ------------------
tsave0 = 0.;             % Tempo inicial de grava��o
tsave = tsave0;          % Tempo de grava��o
npt = 40000;             % Dimens�o do vetor de sa�da de dados
hsave = (tf-tsave0)/npt; % Passo de de grava��o dos vetores de sa�da de dados

if hsave < h             % Sendo o Passo de grava��o menor que o passo de c�lculo (Satura��o)
    hsave = h;           % Defina o passo de grava��o = passo de c�lculo
    npt = (tf-tsave0)/hsave; % Recalcule o a dimens�o dos vetores de sa�da de dados
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
contt = cont1 + cont2;           % Quantidade total de pontos no per�odo

dc = cont1/contt;                % Duty cycle
tint = (tf-t0) * cont1/contt;    % Tempo, dentro do per�odo (tf-t0) que a chave permanece ligada

Vmed = (E)*dc - (E/2)            % Tens�o m�dia
Vref = (E/2)*cos(60*2*pi*(t0+(tf-t0)/2)) % Tens�o de refer�ncia calculada no tempo intermedi�rio do per�odo

plot(Ts,VTs,Ts,V_refs,'r-',[t0 tf],[Vmed Vmed],'k-')
legend(' Tens�o da portadora triangular','Tens�o de refer�ncia','Tens�o M�dia','Location','SouthEast')
xlabel("Tempo (s)")
ylabel("Tens�o (V)")
grid()
