clear;close all;clc
format long

R = 50; %Ohm
L = 0.05; %H
E = 10; %V
V = 20; %V

f = 60; %Hz
T = 1/f;
w = 2*pi*f; %rad/s

CT = L/R; %Constante de tempo do sistema

t = 0; %Tempo inicial de simula��o
h  = 1E-6; %Incremento do tempo de simula��o
tf = 10*CT; %Tempo final de simula��o (10x Constante de tempo)

x = t+h:h:tf;

ii = 0; %Corrente inicial
n = 1; %Indice dos vetores de sa�da

while t < tf
    vv = V;
    ii = (1-h/CT)*ii + (h/L)*(V-E);
    Ts(n) = t;
    vss(n) = vv;
    iss(n) = ii;
    n=n+1;
    t = t + h;
end

figure(1)

plot(Ts.*1000,iss.*1000,'r-','LineWidth',1.5)
title('Integra��o Num�rica - Circuito RLE')
xlabel("Tempo (ms)")
ylabel("Corrente (mA)")
axis([0 10 0 250])
grid()