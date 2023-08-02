function p_thd = thdf(Sinal, Fs, F_fundamental)
N = length(Sinal);
k = 0:N-1;
T = N/Fs;
freq = k/T;
X = fftn(Sinal)*2/N;
cutOff = ceil(N/2);
freq = freq(1:cutOff);
X = X(1:cutOff);

p_thd = 0;
ponto_inicial = round(F_fundamental/(freq(2)-freq(1))) + 1; 

for j = ponto_inicial + 1:(length(X))
    p_thd = p_thd + (abs(X(j))^2);
end
p_thd = sqrt(p_thd)/abs(X(ponto_inicial));
p_thd = p_thd*100;