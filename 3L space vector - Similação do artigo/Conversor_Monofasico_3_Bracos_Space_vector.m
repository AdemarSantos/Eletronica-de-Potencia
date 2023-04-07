clear;close all;clc

% %---------------- Par�metros do Circuito RLC-RL ------------------
E = 127;                        % Tens�o do barramento CC

Vl_ref = 0.95*E;                % Amplitude da tens�o de refer�ncia bra�o L

fg = 60;                        % Frequ�ncia da rede
fl = fg;                        % Frequ�ncia da tens�o sobre a carga
w = 2*pi*fg;                    % Frequ�ncia angular

Pl = 1000;                      % Pot�ncia ativa da carga
fpl = 0.95;                     % Fator de pot�ncia da carga

Nl = Pl/fpl;                    % Pot�ncia aparente da carga
Sl = Nl*exp(j*acos(fpl));       % Pot�ncia complexa da carga
Ql = imag(Sl);                  % Pot�ncia reativa da carga

Zl = (Vl_ref^2)/conj(Sl);       % Imped�ncia da carga
Rl = real(Zl);                  % Resist�ncia da carga
Xl = imag(Zl);                  % Reat�ncia da carga
Ll = Xl/(w);                    % Indut�ncia da carga

Zb = (E^2)/Pl;                  % Imped�ncia do barramento

Xg = 0.2*Zb;                    % Reat�ncia da rede
Rg = 0.02*Zb;                   % Resist�ncia da rede
Lg = Xg/(2*pi*fg);              % Indut�ncia da rede

% C�lculo do valor eficaz da corrente drenada da rede
Ig = roots([Rg -E Pl]);
Ig = min(Ig);

faseg = (180/pi)*atan( (-Ig*Xg)/(E - Ig*Rg) );
Vg_ref = (-Ig*Xg)/(sin(faseg*pi/180));

m = 1;                          % �ndice de modula��o
E = (max([sqrt(2)*Vg_ref sqrt(2)*Vl_ref]))/m;

%---------------- Condi��es do Sistema ------------------
u = 0;                          % Defini��o do fator de reparti��o

ig = 0;                         % Corrente inicial no ramo g
il = 0;                         % Corrente inicial no ramo l

va0_int = 0;                    % valor inicial da integral da tens�o no polo a
vg0_int = 0;                    % valor inicial da integral da tens�o no polo g
vl0_int = 0;                    % valor inicial da integral da tens�o no polo l

%---------------- Par�metros de Simula��o ------------------
h = 1E-6;                       % Passo de c�lculo
t = 0;                          % Tempo inicial de simula��o
tf = 60/60;

%---------------- Par�metros de Grava��o ------------------
tsave0 = 0.;                    % Tempo inicial de grava��o
tsave = tsave0;                 % Tempo de grava��o
npt = 50000;                    % Dimens�o do vetor de sa�da de dados
hsave = (tf-tsave0)/npt;        % Passo de de grava��o dos vetores de sa�da de dados

if hsave < h                    % Sendo o Passo de grava��o menor que o passo de c�lculo (Satura��o)
    hsave = h;                  % Defina o passo de grava��o = passo de c�lculo
    npt = (tf-tsave0)/hsave;    % Recalcule o a dimens�o dos vetores de sa�da de dados
end
n = 0;                          % Inicializa��o da vari�vel de posi��o dos vetores de sa�da

hsave = h;

%---------------- Sinal para c�lculo do valor m�dio ------------------
t_s = 0;
f_s = 10E3;                     % Frequ�ncia do c�lculo do valor m�dio
h_s = 1/f_s;                    % Per�odo do c�lculo do valor m�dio 

tc = 0;

%---------------- Configura��o dos estados das chaves ------------------
q000 = [0 0 0];
q001 = [0 0 1];
q010 = [0 1 0];
q011 = [0 1 1];
q100 = [1 0 0];
q101 = [1 0 1];
q110 = [1 1 0];
q111 = [1 1 1];

%---------------- Configura��o dos estados das tens�es ------------------
v000 = [0 0];           % Vg = 0, Vl = 0
v001 = [0 E];           % Vg = 0, Vl = E
v010 = [E 0];           % Vg = E, Vl = 0
v011 = [E E];           % Vg = E, Vl = E
v100 = [-E -E];         % Vg =-E, Vl =-E
v101 = [-E 0];          % Vg =-E, Vl = 0
v110 = [0 -E];          % Vg = 0, Vl =-E
v111 = [0 0];           % Vg = 0, Vl = 0

while t<tf
    t = t + h;
    
    % Onda triangular
    if t >= t_s
        t_s = t_s + h_s;
        
        % Tens�es m�dias nos bra�os
        va0_med = va0_int/h_s;
        vg0_med = vg0_int/h_s;
        vl0_med = vl0_int/h_s;
        
        va0_int = 0;
        vg0_int = 0;
        vl0_int = 0;
        
        % Tens�es de refer�ncia
        eg = E*sqrt(2)*cos(2*pi*60*t);      % Tens�o de entrada
        vg_ref = sqrt(2)*Vg_ref*cos(2*pi*fg*t + faseg*pi/180);
        vl_ref = sqrt(2)*Vl_ref*cos(2*pi*fl*t);
        
        il1 = (sqrt(2)*(Vl_ref/abs(Zl)))*cos(2*pi*fl*t - acos(fpl) );
        
        q7 = q111;
        q0 = q000;
        
        % Defini��o da regi�o onde ocorre o chaveamento
        if vg_ref > vl_ref && vg_ref > 0 && vl_ref >= 0
            k = 1;
            
            tn = (vg_ref - vl_ref)*(h_s/E); % Tempo de perman�ncia na configura��o n das chaves
            tn1 = vl_ref * (h_s/E);         % Tempo de perman�ncia na configura��o n+1 das chaves
            to = h_s - (tn + tn1);          % Tempo restante dentro do per�odo de chaveamento
            t7 = u * to;                          % Tempo de perman�ncia na configura��o Q111 
            t0 = (1 - u)*to;                      % Tempo de perman�ncia na configura��o Q000
            
            qn  = q100;                           
            qn1 = q110;
            
        elseif vg_ref <= vl_ref && vg_ref > 0 && vl_ref > 0
            k = 2;
            
            tn = vg_ref * (h_s/E);            
            tn1 = (vl_ref - vg_ref)*(h_s/E);         
            
            to = h_s - (tn + tn1);          
            t7 = u * to;                          
            t0 = (1 - u)*to;                     
            
            qn  = q110;
            qn1 = q010;
            
        elseif vg_ref <= 0 && vl_ref > 0
            k = 3;
            
            tn = vl_ref*(h_s/E);           
            tn1 = -vg_ref*(h_s/E);
            
            to = h_s - (tn + tn1);          
            t7 = u * to;                          
            t0 = (1 - u)*to;                     
            
            qn  = q010;
            qn1 = q011;            
            
        elseif vg_ref < vl_ref && vg_ref < 0 && vl_ref <= 0   
            k = 4;
            
            tn = (vl_ref-vg_ref)*(h_s/E);           
            tn1 = -vl_ref*(h_s/E);
            
            to = h_s - (tn + tn1);          
            t7 = u * to;                          
            t0 = (1 - u)*to;
            
            qn  = q011;
            qn1 = q001; 
            
        elseif vg_ref >= vl_ref && vg_ref < 0 && vl_ref < 0   
            k = 5;
            tn = -vg_ref*(h_s/E);          
            tn1 = (vg_ref-vl_ref)*(h_s/E);
            
            to = h_s - (tn + tn1);          
            t7 = u * to;                          
            t0 = (1 - u)*to;
            
            qn  = q001;
            qn1 = q101;
            
        elseif vg_ref >= 0 && vl_ref < 0   
            k = 6;
            tn = -vl_ref*(h_s/E);          
            tn1 = vg_ref*(h_s/E);
            
            to = h_s - (tn + tn1);          
            t7 = u * to;                          
            t0 = (1 - u)*to;
            
            qn  = q101;
            qn1 = q100;            
        end
    end
            
    if tc < t0                  % Tempo da chave na configura��o q111
        q = q7;
    elseif tc < t0 + tn         % Tempo da chave na configura��o qn    
        q = qn;
    elseif tc < t0 + tn + tn1   % Tempo da chave na configura��o qn+1
        q = qn1;
    else
        q = q0;                 % Tempo da chave na configura��o q000
    end

    tc = mod(t,h_s);
    
    % Estado das chaves
    qg = q(1);
    ql = q(2);
    qa = q(3);
    
    % Tens�es de polo
    va0 = (2*qa - 1)*(E/2);
    vg0 = (2*qg - 1)*(E/2);
    vl0 = (2*ql - 1)*(E/2);
    
    % Integra��o das tens�es de polo
    va0_int = va0_int + va0*h;
    vg0_int = vg0_int + vg0*h;
    vl0_int = vl0_int + vl0*h;
    

    vg = vg0 - va0;
    vl = vl0 - va0;
    ig = ig*(1 - h*Rg/Lg) + (h/Lg)*(eg - vg); % Corrente do ramo g
    il = il*(1 - h*Rl/Ll) + (h/Ll)*vl;        % Corrente do ramo l   
    
    if tsave <= t
        tsave = tsave + hsave;
        n = n + 1;
        Ts(n) = t;
        va0s(n) = va0;
        vg0s(n) = vg0;
        vl0s(n) = vl0;
        vgs(n) = vg;
        vls(n) = vl;
        igs(n) = ig;
        ils(n) = il;
        il1s(n) = il1;
        vg_refs(n) = vg_ref;
        vl_refs(n) = vl_ref;
        va0_meds(n) = va0_med;
        vg0_meds(n) = vg0_med;
        vl0_meds(n) = vl0_med;
    end
end

figure(1)
subplot(2,1,1)
plot(Ts,vgs,Ts,vg0_meds-va0_meds,Ts,vg_refs,'r'), zoom
legend('Tens�o de polo', 'Tens�o m�dia', 'Tens�o de refer�ncia')
title('Bra�o G')
subplot(2,1,2)
plot(Ts,vls,Ts,vl0_meds-va0_meds,Ts,vl_refs,'r'), zoom
legend('Tens�o de polo', 'Tens�o m�dia', 'Tens�o de refer�ncia')
title('Bra�o L')

figure('Name','Corrente do circuito: Lado G')
plot(Ts,igs,'r-','LineWidth',1), zoom
title('Corrente do circuito: Lado G')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()

figure('Name','Corrente do circuito: Lado G')
plot(Ts,ils, Ts,il1s,'r-','LineWidth',1), zoom
title('Corrente do circuito: Lado L')
xlabel("Tempo (s)")
ylabel("Corrente (A)")
grid()
